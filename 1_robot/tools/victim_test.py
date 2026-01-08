from __future__ import annotations

import os
import time
from pathlib import Path
from dataclasses import dataclass

import cv2
import numpy as np
from libcamera import Transform
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics

from hardware.camera import Camera


# -------------------------
# Defaults (match your test)
# -------------------------
DEFAULT_SCORE_THRESH = 0.30
DEFAULT_MAX_DETECTIONS = 50

# If your model filename is just network.rpk (not containing "yolo"),
# force these defaults because YOLO style exports usually expect them.
FORCE_BBOX_NORMALIZATION = True
FORCE_BBOX_ORDER = "xy"  # YOLO usually outputs x0,y0,x1,y1

# "auto", "xyxy", "xywh"
BBOX_FORMAT = "auto"

# Match your test orientation
DEFAULT_TRANSFORM = Transform(hflip=1, vflip=1)


def _maybe_softmax(x: np.ndarray) -> np.ndarray:
    x = x - np.max(x, axis=-1, keepdims=True)
    ex = np.exp(x)
    return ex / np.clip(np.sum(ex, axis=-1, keepdims=True), 1e-9, None)


def _infer_bbox_format(boxes: np.ndarray) -> str:
    if boxes.shape[1] != 4:
        return "xyxy"
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    frac_bad = float(np.mean((x2 < x1) | (y2 < y1)))
    return "xywh" if frac_bad > 0.35 else "xyxy"


def _boxes_to_yx_yx(
    boxes: np.ndarray,
    intrinsics: NetworkIntrinsics,
    imx500: IMX500,
) -> np.ndarray:
    boxes = boxes.astype(np.float32)

    order = getattr(intrinsics, "bbox_order", "yx")
    fmt = BBOX_FORMAT
    if fmt == "auto":
        fmt = _infer_bbox_format(boxes)

    # convert xywh -> xyxy (or yxhw -> yxyx)
    if fmt == "xywh":
        if order == "xy":
            x0, y0, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
            boxes = np.stack([x0, y0, x0 + w, y0 + h], axis=1)
        else:
            y0, x0, h, w = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
            boxes = np.stack([y0, x0, y0 + h, x0 + w], axis=1)

    # normalize if requested
    if getattr(intrinsics, "bbox_normalization", False):
        in_w, in_h = imx500.get_input_size()
        if order == "xy":
            boxes[:, [0, 2]] /= float(in_w)
            boxes[:, [1, 3]] /= float(in_h)
        else:
            boxes[:, [0, 2]] /= float(in_h)
            boxes[:, [1, 3]] /= float(in_w)

    # ensure yx order for convert_inference_coords
    if order == "xy":
        boxes = boxes[:, [1, 0, 3, 2]]

    return boxes


def _load_labels_from_txt(txt_path: Path) -> list[str] | None:
    if not txt_path.exists():
        return None
    labs = [ln.strip() for ln in txt_path.read_text().splitlines() if ln.strip()]
    return labs if labs else None


def _get_labels(intrinsics: NetworkIntrinsics, model_path: str) -> list[str] | None:
    labels = getattr(intrinsics, "labels", None)
    if labels:
        labels = list(labels)
        if getattr(intrinsics, "ignore_dash_labels", False):
            labels = [lab for lab in labels if lab and lab != "-"]
        if labels:
            return labels

    p = Path(model_path)
    labs = _load_labels_from_txt(p.with_suffix(".txt"))
    if labs:
        return labs
    labs2 = _load_labels_from_txt(p.parent / "labels.txt")
    if labs2:
        return labs2

    return None


def _apply_bbox_defaults(intrinsics: NetworkIntrinsics) -> None:
    if FORCE_BBOX_NORMALIZATION is not None:
        setattr(intrinsics, "bbox_normalization", bool(FORCE_BBOX_NORMALIZATION))
    if FORCE_BBOX_ORDER is not None:
        setattr(intrinsics, "bbox_order", str(FORCE_BBOX_ORDER))


def parse_detections(
    np_outputs,
    intrinsics: NetworkIntrinsics,
    imx500: IMX500,
    metadata,
    picam2,
    score_thresh: float,
    max_dets: int,
) -> list[tuple[float, float, float, float, int, float]]:
    if np_outputs is None or len(np_outputs) < 2:
        return []

    boxes = np.asarray(np_outputs[0])
    if boxes.ndim == 3:
        boxes = boxes[0]
    if boxes.ndim != 2 or boxes.shape[1] != 4:
        return []

    N = boxes.shape[0]

    # Case A: 3 outputs -> scores + classes
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

    # Case B: 2 outputs -> class scores (N, nc)
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

    boxes_yxyx = _boxes_to_yx_yx(boxes, intrinsics, imx500)

    dets: list[tuple[float, float, float, float, int, float]] = []
    for i in range(len(scores)):
        sc = float(scores[i])
        if sc < score_thresh:
            continue
        if len(dets) >= max_dets:
            break

        y0, x0, y1, x1 = [float(v) for v in boxes_yxyx[i]]
        cls = int(classes[i])

        x, y, w, h = imx500.convert_inference_coords((y0, x0, y1, x1), metadata, picam2)
        dets.append((float(x), float(y), float(w), float(h), cls, sc))

    return dets


@dataclass
class VictimModel:
    model_path: str
    imx500: IMX500
    intrinsics: NetworkIntrinsics
    labels: list[str] | None
    input_w: int
    input_h: int
    fps: int

    @classmethod
    def load(cls, model_path: str) -> "VictimModel":
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Victim model not found: {model_path}")

        imx500 = IMX500(model_path)
        intr = imx500.network_intrinsics or NetworkIntrinsics()
        _apply_bbox_defaults(intr)

        labels = _get_labels(intr, model_path)

        in_w, in_h = imx500.get_input_size()

        fps = 30
        if getattr(intr, "inference_rate", None):
            try:
                fps = int(intr.inference_rate)
            except Exception:
                pass

        return cls(
            model_path=model_path,
            imx500=imx500,
            intrinsics=intr,
            labels=labels,
            input_w=int(in_w),
            input_h=int(in_h),
            fps=int(fps),
        )


class VictimDetector:
    """
    Uses the existing evac Camera's Picamera2 instance, but:
      - stops the Camera background capture thread
      - reconfigures for IMX500 main+lores streams
      - runs capture_request() to get metadata for imx500.get_outputs(...)
    """

    def __init__(
        self,
        evac_cam: Camera,
        model: VictimModel,
        window_name: str = "EVAC",
        display_size: tuple[int, int] = (800, 600),
        transform: Transform = DEFAULT_TRANSFORM,
        score_thresh: float = DEFAULT_SCORE_THRESH,
        max_detections: int = DEFAULT_MAX_DETECTIONS,
        period_s: float = 0.10,
    ) -> None:
        self.evac_cam = evac_cam
        self.picam2 = evac_cam.camera
        self.model = model

        self.window_name = window_name
        self.display_w, self.display_h = display_size

        self.score_thresh = float(score_thresh)
        self.max_detections = int(max_detections)

        self.period_s = float(period_s)
        self._next_infer = 0.0

        self.last_dets: list[tuple[float, float, float, float, int, float]] = []
        self.none_count = 0

        self._fps_smooth = 0.0
        self._t_prev = time.perf_counter()

        # IMPORTANT: stop Camera thread, we will use capture_request directly
        try:
            self.evac_cam.stop()
        except Exception:
            pass

        config = self.picam2.create_preview_configuration(
            main={"size": (self.model.input_w, self.model.input_h), "format": "RGB888"},
            lores={"size": (self.model.input_w, self.model.input_h), "format": "YUV420"},
            transform=transform,
            controls={"FrameRate": self.model.fps},
            buffer_count=4,
        )
        self.picam2.configure(config)
        self.picam2.start()

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def close(self) -> None:
        try:
            self.picam2.stop()
        except Exception:
            pass
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass

    def step(self) -> None:
        req = self.picam2.capture_request()
        try:
            frame = req.make_array("main")
            meta = req.get_metadata()

            now = time.perf_counter()
            np_outputs = None

            if now >= self._next_infer:
                np_outputs = self.model.imx500.get_outputs(meta, add_batch=True)
                self._next_infer = now + self.period_s

                if np_outputs is None:
                    self.none_count += 1
                else:
                    self.last_dets = parse_detections(
                        np_outputs,
                        self.model.intrinsics,
                        self.model.imx500,
                        meta,
                        self.picam2,
                        self.score_thresh,
                        self.max_detections,
                    )

            dets = self.last_dets
            H, W = frame.shape[:2]

            # top list
            y0 = 80
            line_h = 22
            cv2.putText(
                frame,
                f"Top dets: {len(dets)}",
                (10, y0 - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            show_n = min(5, len(dets))
            for i in range(show_n):
                x, y, w, h, cls_id, score = dets[i]
                cls_id = int(cls_id)
                name = (
                    self.model.labels[cls_id]
                    if (self.model.labels and 0 <= cls_id < len(self.model.labels))
                    else str(cls_id)
                )
                cv2.putText(
                    frame,
                    f"{i+1}) {name} {score:.2f}",
                    (10, y0 + i * line_h),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            # boxes
            for (x, y, w, h, cls_id, score) in dets:
                x = int(round(x))
                y = int(round(y))
                w = int(round(w))
                h = int(round(h))

                x1 = max(0, min(W - 1, x))
                y1 = max(0, min(H - 1, y))
                x2 = max(0, min(W - 1, x + max(1, w)))
                y2 = max(0, min(H - 1, y + max(1, h)))

                if x2 <= x1 or y2 <= y1:
                    continue

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                cls_id = int(cls_id)
                name = (
                    self.model.labels[cls_id]
                    if (self.model.labels and 0 <= cls_id < len(self.model.labels))
                    else str(cls_id)
                )
                cv2.putText(
                    frame,
                    f"{name} {float(score):.2f}",
                    (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            # fps overlay
            t_now = time.perf_counter()
            dt = t_now - self._t_prev
            self._t_prev = t_now
            inst = (1.0 / dt) if dt > 0 else 0.0
            self._fps_smooth = inst if self._fps_smooth == 0.0 else (0.9 * self._fps_smooth + 0.1 * inst)

            outs_n = 0 if np_outputs is None else len(np_outputs)
            cv2.putText(
                frame,
                f"FPS {self._fps_smooth:.1f}  det {len(dets)}  outs {outs_n}  none {self.none_count}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            disp = cv2.resize(frame, (self.display_w, self.display_h), interpolation=cv2.INTER_AREA)
            cv2.imshow(self.window_name, disp)
            cv2.waitKey(1)

        finally:
            req.release()
