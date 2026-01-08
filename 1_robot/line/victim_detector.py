from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time as _time

import cv2
import numpy as np
from libcamera import Transform
from picamera2 import Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics


# Same defaults that worked in your victim_test.py
_FORCE_BBOX_NORMALIZATION = True
_FORCE_BBOX_ORDER = "xy"


@dataclass
class Detection:
    x: int
    y: int
    w: int
    h: int
    cls: int
    score: float


def _load_labels_txt(p: Path) -> list[str] | None:
    if not p.exists():
        return None
    labs = [ln.strip() for ln in p.read_text().splitlines() if ln.strip()]
    return labs or None


def _get_labels(intrinsics: NetworkIntrinsics, model_path: str) -> list[str] | None:
    labels = getattr(intrinsics, "labels", None)
    if labels:
        labels = list(labels)
        if getattr(intrinsics, "ignore_dash_labels", False):
            labels = [lab for lab in labels if lab and lab != "-"]
        if labels:
            return labels

    mp = Path(model_path)
    return _load_labels_txt(mp.with_suffix(".txt")) or _load_labels_txt(mp.parent / "labels.txt")


def _maybe_softmax(x: np.ndarray) -> np.ndarray:
    x = x - np.max(x, axis=-1, keepdims=True)
    ex = np.exp(x)
    return ex / np.clip(np.sum(ex, axis=-1, keepdims=True), 1e-9, None)


def _infer_bbox_format(boxes: np.ndarray) -> str:
    if boxes.shape[1] != 4:
        return "xyxy"
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    frac_bad = float(np.mean((x2 < x1) | (y2 < y1)))
    return "xywh" if frac_bad > 0.35 else "xyxy"


def _boxes_to_yx_yx(boxes: np.ndarray, intrinsics: NetworkIntrinsics, imx500: IMX500) -> np.ndarray:
    boxes = boxes.astype(np.float32)

    order = getattr(intrinsics, "bbox_order", "yx")
    fmt = _infer_bbox_format(boxes)

    if fmt == "xywh":
        if order == "xy":
            x0, y0, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
            boxes = np.stack([x0, y0, x0 + w, y0 + h], axis=1)
        else:
            y0, x0, h, w = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
            boxes = np.stack([y0, x0, y0 + h, x0 + w], axis=1)

    if getattr(intrinsics, "bbox_normalization", False):
        in_w, in_h = imx500.get_input_size()
        if order == "xy":
            boxes[:, [0, 2]] /= float(in_w)
            boxes[:, [1, 3]] /= float(in_h)
        else:
            boxes[:, [0, 2]] /= float(in_h)
            boxes[:, [1, 3]] /= float(in_w)

    if order == "xy":
        boxes = boxes[:, [1, 0, 3, 2]]  # yx order

    return boxes


class VictimModel:
    """
    Owns the IMX500 + Picamera2 pipeline.
    Use start() in mode 2, stop() when leaving mode 2.
    """

    def __init__(self, model_path: str, *, transform: Transform | None = None) -> None:
        self.model_path = model_path
        self.transform = transform or Transform(hflip=1, vflip=1)

        self.imx500 = IMX500(model_path)
        self.intrinsics = self.imx500.network_intrinsics or NetworkIntrinsics()

        setattr(self.intrinsics, "bbox_normalization", bool(_FORCE_BBOX_NORMALIZATION))
        setattr(self.intrinsics, "bbox_order", str(_FORCE_BBOX_ORDER))

        self.labels = _get_labels(self.intrinsics, model_path)
        self.picam2: Picamera2 | None = None

    @classmethod
    def load(cls, model_path: str) -> "VictimModel":
        return cls(model_path)

    def start(self) -> None:
        if self.picam2 is not None:
            return

        self.picam2 = Picamera2(self.imx500.camera_num)
        in_w, in_h = self.imx500.get_input_size()

        fps = 30
        if getattr(self.intrinsics, "inference_rate", None):
            try:
                fps = int(self.intrinsics.inference_rate)
            except Exception:
                pass

        config = self.picam2.create_preview_configuration(
            main={"size": (in_w, in_h), "format": "RGB888"},
            lores={"size": (in_w, in_h), "format": "YUV420"},
            transform=self.transform,
            controls={"FrameRate": fps},
            buffer_count=4,
        )
        self.picam2.configure(config)
        self.picam2.start()

    def stop(self) -> None:
        if self.picam2 is None:
            return
        try:
            self.picam2.stop()
        except Exception:
            pass
        self.picam2 = None

    def capture(self) -> tuple[np.ndarray | None, dict | None]:
        if self.picam2 is None:
            return None, None

        req = self.picam2.capture_request()
        try:
            frame = req.make_array("main")
            meta = req.get_metadata()
            return frame, meta
        finally:
            req.release()

    def outputs_from_meta(self, meta: dict):
        return self.imx500.get_outputs(meta, add_batch=True)


class VictimDetector:
    """
    Pure processing.
    Input: (frame, meta)
    Output: (display_frame_with_boxes, detections_in_display_coords)
    """

    def __init__(
        self,
        model: VictimModel,
        *,
        crop_top_px: int = 0,
        score_thresh: float = 0.30,
        max_dets: int = 50,
        display_size: tuple[int, int] = (800, 600),
    ) -> None:
        self.model = model
        self.crop_top_px = int(crop_top_px)
        self.score_thresh = float(score_thresh)
        self.max_dets = int(max_dets)
        self.display_size = display_size

        self._last_dets: list[Detection] = []
        self._none_count = 0
        self._fps_smooth = 0.0
        self._t_prev = _time.perf_counter()

    def _parse(self, np_outputs, meta: dict) -> list[Detection]:
        if np_outputs is None or len(np_outputs) < 2:
            return []

        boxes = np.asarray(np_outputs[0])
        if boxes.ndim == 3:
            boxes = boxes[0]
        if boxes.ndim != 2 or boxes.shape[1] != 4:
            return []

        N = boxes.shape[0]

        if len(np_outputs) >= 3:
            scores = np.asarray(np_outputs[1])
            classes = np.asarray(np_outputs[2])
            if scores.ndim == 3:
                scores = scores[0]
            if classes.ndim == 3:
                classes = classes[0]
            scores = scores.reshape(-1).astype(np.float32)
            classes = classes.reshape(-1).astype(np.int32)
            n = min(N, len(scores), len(classes))
            boxes = boxes[:n]
            scores = scores[:n]
            classes = classes[:n]
        else:
            cls_scores = np.asarray(np_outputs[1])
            if cls_scores.ndim == 3:
                cls_scores = cls_scores[0]
            if cls_scores.ndim != 2:
                return []
            n = min(N, cls_scores.shape[0])
            boxes = boxes[:n]
            cls_scores = cls_scores[:n].astype(np.float32)

            row_sum = float(np.mean(np.sum(cls_scores, axis=1)))
            if not (0.8 <= row_sum <= 1.2):
                cls_scores = _maybe_softmax(cls_scores)

            classes = np.argmax(cls_scores, axis=1).astype(np.int32)
            scores = cls_scores[np.arange(n), classes].astype(np.float32)

        boxes_yxyx = _boxes_to_yx_yx(boxes, self.model.intrinsics, self.model.imx500)

        dets: list[Detection] = []
        for i in range(len(scores)):
            sc = float(scores[i])
            if sc < self.score_thresh:
                continue
            if len(dets) >= self.max_dets:
                break

            y0, x0, y1, x1 = [float(v) for v in boxes_yxyx[i]]
            cls = int(classes[i])

            x, y, w, h = self.model.imx500.convert_inference_coords(
                (y0, x0, y1, x1), meta, self.model.picam2
            )
            dets.append(Detection(int(round(x)), int(round(y)), int(round(w)), int(round(h)), cls, sc))

        return dets

    def process(self, frame: np.ndarray, meta: dict | None) -> tuple[np.ndarray, list[Detection]]:
        if meta is None:
            disp = frame[self.crop_top_px :, :] if self.crop_top_px > 0 else frame
            return self._finalize_display(disp), []

        np_outputs = self.model.outputs_from_meta(meta)

        if np_outputs is None:
            self._none_count += 1
            dets_full = self._last_dets
        else:
            dets_full = self._parse(np_outputs, meta)
            self._last_dets = dets_full

        # FPS
        t_now = _time.perf_counter()
        dt = t_now - self._t_prev
        self._t_prev = t_now
        inst = (1.0 / dt) if dt > 0 else 0.0
        self._fps_smooth = inst if self._fps_smooth == 0.0 else (0.9 * self._fps_smooth + 0.1 * inst)

        crop = self.crop_top_px
        base = frame[crop:, :] if crop > 0 else frame
        out = base.copy()
        H, W = out.shape[:2]

        dets: list[Detection] = []
        for d in dets_full:
            # shift into cropped coordinates
            y = d.y - crop
            if y + d.h <= 0:
                continue
            if y >= H:
                continue

            x1 = max(0, min(W - 1, d.x))
            y1 = max(0, min(H - 1, y))
            x2 = max(0, min(W - 1, d.x + max(1, d.w)))
            y2 = max(0, min(H - 1, y + max(1, d.h)))
            if x2 <= x1 or y2 <= y1:
                continue

            dets.append(Detection(x1, y1, x2 - x1, y2 - y1, d.cls, d.score))

        # overlay
        cv2.putText(
            out,
            f"FPS {self._fps_smooth:.1f} det {len(dets)} none {self._none_count}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # draw boxes
        for d in dets:
            cv2.rectangle(out, (d.x, d.y), (d.x + d.w, d.y + d.h), (0, 255, 0), 2)
            name = (
                self.model.labels[d.cls]
                if (self.model.labels and 0 <= d.cls < len(self.model.labels))
                else str(d.cls)
            )
            cv2.putText(
                out,
                f"{name} {d.score:.2f}",
                (d.x, max(0, d.y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return self._finalize_display(out), dets

    def _finalize_display(self, img: np.ndarray) -> np.ndarray:
        if self.display_size:
            return cv2.resize(img, self.display_size, interpolation=cv2.INTER_AREA)
        return img
