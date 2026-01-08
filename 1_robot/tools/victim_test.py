#!/usr/bin/env python3
import os
import time
from pathlib import Path

import cv2
import numpy as np
from libcamera import Transform
from picamera2 import Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics

# =========================
# Hardcoded settings
# =========================
MODEL_PATH = "/home/fusion/FusionZero-Robocup-2026/imx_model_out/network.rpk"

DISPLAY_W, DISPLAY_H = 800, 600

SCORE_THRESH = 0.30
MAX_DETECTIONS = 50

TRANSFORM = Transform(hflip=1, vflip=1)

# "auto", "xyxy", "xywh"
BBOX_FORMAT = "auto"

LABELS_TXT_FALLBACK = True

# If your model filename is just network.rpk (not containing "yolo"),
# force these defaults because YOLO style exports usually expect them.
FORCE_BBOX_NORMALIZATION = True   # set False if your boxes are already 0..1
FORCE_BBOX_ORDER = "xy"           # YOLO usually outputs x0,y0,x1,y1


def load_labels_from_txt(txt_path: Path):
    if not txt_path.exists():
        return None
    labs = [ln.strip() for ln in txt_path.read_text().splitlines() if ln.strip()]
    return labs if labs else None


def apply_bbox_defaults(intrinsics: NetworkIntrinsics):
    if FORCE_BBOX_NORMALIZATION is not None:
        setattr(intrinsics, "bbox_normalization", bool(FORCE_BBOX_NORMALIZATION))
    if FORCE_BBOX_ORDER is not None:
        setattr(intrinsics, "bbox_order", str(FORCE_BBOX_ORDER))


def get_labels(intrinsics: NetworkIntrinsics, model_path: str):
    labels = getattr(intrinsics, "labels", None)
    if labels:
        labels = list(labels)
        if getattr(intrinsics, "ignore_dash_labels", False):
            labels = [lab for lab in labels if lab and lab != "-"]
        if labels:
            return labels

    if LABELS_TXT_FALLBACK:
        p = Path(model_path)
        labs = load_labels_from_txt(p.with_suffix(".txt"))
        if labs:
            return labs
        labs2 = load_labels_from_txt(p.parent / "labels.txt")
        if labs2:
            return labs2

    return None


def _maybe_softmax(x: np.ndarray):
    x = x - np.max(x, axis=-1, keepdims=True)
    ex = np.exp(x)
    return ex / np.clip(np.sum(ex, axis=-1, keepdims=True), 1e-9, None)


def _infer_bbox_format(boxes: np.ndarray):
    if boxes.shape[1] != 4:
        return "xyxy"
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    frac_bad = float(np.mean((x2 < x1) | (y2 < y1)))
    return "xywh" if frac_bad > 0.35 else "xyxy"


def _boxes_to_yx_yx(boxes: np.ndarray, intrinsics: NetworkIntrinsics, imx500: IMX500):
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


def parse_detections(np_outputs, intrinsics: NetworkIntrinsics, imx500: IMX500, metadata, picam2: Picamera2):
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

        # If it looks like logits, softmax it
        row_sum = float(np.mean(np.sum(cls_scores, axis=1)))
        if not (0.8 <= row_sum <= 1.2):
            cls_scores = _maybe_softmax(cls_scores)

        classes = np.argmax(cls_scores, axis=1).astype(np.int32)
        scores = cls_scores[np.arange(n), classes].astype(np.float32)

    # Convert boxes properly (format + order + optional normalization)
    boxes_yxyx = _boxes_to_yx_yx(boxes, intrinsics, imx500)

    dets = []
    for i in range(len(scores)):
        sc = float(scores[i])
        if sc < SCORE_THRESH:
            continue
        if len(dets) >= MAX_DETECTIONS:
            break

        y0, x0, y1, x1 = [float(v) for v in boxes_yxyx[i]]
        cls = int(classes[i])

        x, y, w, h = imx500.convert_inference_coords((y0, x0, y1, x1), metadata, picam2)
        dets.append((x, y, w, h, cls, sc))

    return dets


def main():
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model not found:\n{MODEL_PATH}")

    imx500 = IMX500(MODEL_PATH)

    intrinsics = imx500.network_intrinsics or NetworkIntrinsics()
    apply_bbox_defaults(intrinsics)

    labels = get_labels(intrinsics, MODEL_PATH)

    picam2 = Picamera2(imx500.camera_num)
    in_w, in_h = imx500.get_input_size()
    main_w, main_h = in_w, in_h

    fps = 30
    if hasattr(intrinsics, "inference_rate") and intrinsics.inference_rate:
        try:
            fps = int(intrinsics.inference_rate)
        except Exception:
            pass

    config = picam2.create_preview_configuration(
        main={"size": (main_w, main_h), "format": "RGB888"},
        lores={"size": (in_w, in_h), "format": "YUV420"},
        transform=TRANSFORM,
        controls={"FrameRate": fps},
        buffer_count=4,
    )
    picam2.configure(config)
    picam2.start()

    win = "IMX500 RPK Test"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    last_dets = []
    fps_smooth = 0.0
    t_prev = time.perf_counter()
    none_count = 0

    try:
        while True:
            req = picam2.capture_request()
            try:
                frame = req.make_array("main")
                meta = req.get_metadata()

                np_outputs = imx500.get_outputs(meta, add_batch=True)

                if np_outputs is None:
                    none_count += 1
                    dets = last_dets
                else:
                    dets = parse_detections(np_outputs, intrinsics, imx500, meta, picam2)
                    last_dets = dets

                H, W = frame.shape[:2]

                # Always show what it thinks it sees (top 5)
                show_n = min(5, len(dets))
                y0 = 80
                line_h = 22
                cv2.putText(frame, f"Top dets: {len(dets)}", (10, y0 - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

                for i in range(show_n):
                    x, y, w, h, cls, score = dets[i]
                    cls = int(cls)
                    name = labels[cls] if (labels and 0 <= cls < len(labels)) else str(cls)
                    cv2.putText(frame, f"{i+1}) {name} {score:.2f}", (10, y0 + i * line_h),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

                # Draw boxes once, clamped
                for (x, y, w, h, cls, score) in dets:
                    x = int(round(x)); y = int(round(y))
                    w = int(round(w)); h = int(round(h))

                    x1 = max(0, min(W - 1, x))
                    y1 = max(0, min(H - 1, y))
                    x2 = max(0, min(W - 1, x + max(1, w)))
                    y2 = max(0, min(H - 1, y + max(1, h)))

                    if x2 <= x1 or y2 <= y1:
                        continue

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cls = int(cls)
                    name = labels[cls] if (labels and 0 <= cls < len(labels)) else str(cls)
                    cv2.putText(frame, f"{name} {float(score):.2f}", (x1, max(0, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

                # FPS overlay
                t_now = time.perf_counter()
                dt = t_now - t_prev
                t_prev = t_now
                inst = (1.0 / dt) if dt > 0 else 0.0
                fps_smooth = inst if fps_smooth == 0.0 else (0.9 * fps_smooth + 0.1 * inst)

                outs_n = 0 if np_outputs is None else len(np_outputs)
                cv2.putText(frame, f"FPS {fps_smooth:.1f}  det {len(dets)}  outs {outs_n}  none {none_count}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

                if dets: print("sample det:", dets[0], "frame:", W, H)
                disp = cv2.resize(frame, (DISPLAY_W, DISPLAY_H), interpolation=cv2.INTER_AREA)
                cv2.imshow(win, disp)


            finally:
                req.release()

            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord("q")):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
