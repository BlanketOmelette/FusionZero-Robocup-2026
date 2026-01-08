#!/usr/bin/env python3
import os
import time

import cv2
import numpy as np
from libcamera import Transform
from picamera2 import Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics

# =========================
# Hardcoded settings
# =========================
MODEL_PATH = "/home/fusion/FusionZero-Robocup-2026/imx500_models/imx500_network_yolo11n_pp.rpk"

# What you see on screen (purely display, not camera config)
DISPLAY_W, DISPLAY_H = 640, 480

SCORE_THRESH = 0.30
MAX_DETECTIONS = 50

# upside down + also need horizontal flip -> both flips
TRANSFORM = Transform(hflip=1, vflip=1)

COCO80 = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
    "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
    "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
    "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
    "broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed",
    "dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
]

def ensure_yolo_bbox_settings(intrinsics: NetworkIntrinsics, model_path: str):
    # YOLO model zoo examples want bbox-normalization and bbox-order xy
    if "yolo" in os.path.basename(model_path).lower():
        setattr(intrinsics, "bbox_normalization", True)
        setattr(intrinsics, "bbox_order", "xy")

def get_labels(intrinsics: NetworkIntrinsics):
    labels = getattr(intrinsics, "labels", None)
    if labels:
        labels = list(labels)
        if getattr(intrinsics, "ignore_dash_labels", False):
            labels = [lab for lab in labels if lab and lab != "-"]
        return labels
    return COCO80

def parse_detections(np_outputs, intrinsics: NetworkIntrinsics, imx500: IMX500, metadata, picam2: Picamera2):
    if np_outputs is None or len(np_outputs) < 3:
        return []

    boxes = np_outputs[0][0]                        # (N,4)
    scores = np.asarray(np_outputs[1][0]).reshape(-1)
    classes = np.asarray(np_outputs[2][0]).reshape(-1).astype(int)
    if boxes is None or len(boxes) == 0:
        return []

    # If requested, normalize model output from input pixels -> 0..1
    if getattr(intrinsics, "bbox_normalization", False):
        in_w, in_h = imx500.get_input_size()
        boxes = boxes.astype(np.float32)
        if getattr(intrinsics, "bbox_order", "yx") == "xy":
            boxes[:, [0, 2]] /= float(in_w)
            boxes[:, [1, 3]] /= float(in_h)
        else:
            boxes[:, [0, 2]] /= float(in_h)
            boxes[:, [1, 3]] /= float(in_w)

    # convert_inference_coords expects yx ordering
    if getattr(intrinsics, "bbox_order", "yx") == "xy":
        boxes = boxes[:, [1, 0, 3, 2]]

    dets = []
    for i, sc in enumerate(scores):
        sc = float(sc)
        if sc < SCORE_THRESH:
            continue
        if len(dets) >= MAX_DETECTIONS:
            break

        y0, x0, y1, x1 = [float(v) for v in boxes[i]]
        cls = int(classes[i])

        x, y, w, h = imx500.convert_inference_coords((y0, x0, y1, x1), metadata, picam2)
        dets.append((x, y, w, h, cls, sc))

    return dets

def main():
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model not found:\n{MODEL_PATH}")

    # Create IMX500 first
    imx500 = IMX500(MODEL_PATH)

    intrinsics = imx500.network_intrinsics or NetworkIntrinsics()
    ensure_yolo_bbox_settings(intrinsics, MODEL_PATH)
    labels = get_labels(intrinsics)

    # Picamera2 wants a camera index
    picam2 = Picamera2(imx500.camera_num)

    # Model input size (lores) for inference
    in_w, in_h = imx500.get_input_size()

    # IMPORTANT: main must be >= lores, so make main at least the model input size
    main_w, main_h = in_w, in_h

    # Optional: don’t push camera faster than model inference rate if available
    fps = 30
    if hasattr(intrinsics, "inference_rate") and intrinsics.inference_rate:
        try:
            fps = int(intrinsics.inference_rate)
        except Exception:
            pass

    config = picam2.create_preview_configuration(
        main={"size": (main_w, main_h), "format": "RGB888"},   # no cvtColor
        lores={"size": (in_w, in_h), "format": "YUV420"},
        transform=TRANSFORM,
        controls={"FrameRate": fps},
        buffer_count=4,
    )
    picam2.configure(config)
    picam2.start()

    cv2.namedWindow("IMX500 YOLO11", cv2.WINDOW_NORMAL)

    last_dets = []
    fps_smooth = 0.0
    t_prev = time.perf_counter()

    try:
        while True:
            req = picam2.capture_request()
            try:
                frame = req.make_array("main")  # RGB888
                meta = req.get_metadata()

                np_outputs = imx500.get_outputs(meta, add_batch=True)
                if np_outputs is None:
                    dets = last_dets
                else:
                    dets = parse_detections(np_outputs, intrinsics, imx500, meta, picam2)
                    last_dets = dets

                for (x, y, w, h, cls, score) in dets:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    name = labels[cls] if 0 <= cls < len(labels) else str(cls)
                    cv2.putText(frame, f"{name} {score:.2f}", (x, max(0, y - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

                # FPS overlay
                t_now = time.perf_counter()
                dt = t_now - t_prev
                t_prev = t_now
                inst = (1.0 / dt) if dt > 0 else 0.0
                fps_smooth = inst if fps_smooth == 0.0 else (0.9 * fps_smooth + 0.1 * inst)

                cv2.putText(frame, f"FPS {fps_smooth:.1f}  det {len(dets)}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                # Downscale only for display (keeps config valid + speeds up imshow)
                disp = cv2.resize(frame, (DISPLAY_W, DISPLAY_H), interpolation=cv2.INTER_AREA)
                cv2.imshow("IMX500 YOLO11", disp)

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
