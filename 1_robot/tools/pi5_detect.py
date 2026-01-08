#!/usr/bin/env python3
import time
from pathlib import Path

import cv2
import numpy as np
from libcamera import Transform
from picamera2 import Picamera2


# =========================
# Hardcoded settings
# =========================
MODEL_PATH = Path("/home/fusion/FusionZero-Robocup-2026/models/yolov8n.onnx")

# Camera stream size (lower = faster overall)
CAM_W, CAM_H = 640, 480

# YOLO input size (most yolov8n ONNX exports use 640)
INPUT_SIZE = 640

# Hardcoded flip (you wanted both)
TRANSFORM = Transform(hflip=1, vflip=1)

# Thresholds
CONF_THRESH = 0.30
NMS_THRESH = 0.45

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


def pick_imx500_index() -> int:
    infos = Picamera2.global_camera_info()
    if not infos:
        return 0
    for idx, info in enumerate(infos):
        model = str(info.get("Model", "")).lower()
        if "imx500" in model:
            return idx
    return 0


def letterbox(img, new_shape=(640, 640), color=114):
    h, w = img.shape[:2]
    new_h, new_w = new_shape
    ratio = min(new_h / h, new_w / w)
    nh, nw = int(round(h * ratio)), int(round(w * ratio))
    img_resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w = new_w - nw
    pad_h = new_h - nh
    left = pad_w // 2
    top = pad_h // 2
    out = cv2.copyMakeBorder(
        img_resized,
        top, pad_h - top,
        left, pad_w - left,
        cv2.BORDER_CONSTANT,
        value=(color, color, color),
    )
    return out, ratio, left, top


def parse_yolov8_outputs(outputs):
    """
    Handle common YOLOv8 ONNX output shapes.
    OpenCV example notes shapes like (1,25200,85) or (1,84,8400) etc. :contentReference[oaicite:2]{index=2}
    """
    if outputs.ndim == 3 and outputs.shape[1] == 84 and outputs.shape[2] == 8400:
        return outputs[0].T  # (8400,84)
    if outputs.ndim == 3 and outputs.shape[1] == 25200 and outputs.shape[2] == 85:
        return outputs[0]    # (25200,85)
    if outputs.ndim == 3 and outputs.shape[1] == 8400 and outputs.shape[2] == 85:
        return outputs[0]
    raise ValueError(f"Unexpected output shape: {outputs.shape}")


def main():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"ONNX model not found:\n{MODEL_PATH}")

    # Load model
    net = cv2.dnn.readNet(str(MODEL_PATH))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    # Camera (IMX500 used as a normal camera, no IMX500 network)
    cam_index = pick_imx500_index()
    picam2 = Picamera2(cam_index)
    config = picam2.create_preview_configuration(
        main={"size": (CAM_W, CAM_H), "format": "RGB888"},
        transform=TRANSFORM,
        buffer_count=4,
        controls={"FrameRate": 30},
    )
    picam2.configure(config)
    picam2.start()

    cv2.namedWindow("Pi5 CPU YOLO (OpenCV DNN)", cv2.WINDOW_NORMAL)

    fps_smooth = 0.0
    infer_ms_smooth = 0.0
    t_prev = time.perf_counter()

    try:
        while True:
            frame = picam2.capture_array("main")  # RGB888, no cvtColor

            # Preprocess: letterbox to 640x640
            lb, ratio, dw, dh = letterbox(frame, (INPUT_SIZE, INPUT_SIZE))

            # blobFromImage: keep RGB order since frame is already RGB
            blob = cv2.dnn.blobFromImage(lb, 1 / 255.0, (INPUT_SIZE, INPUT_SIZE), swapRB=False, crop=False)
            net.setInput(blob)

            t0 = time.perf_counter()
            outputs = net.forward()
            t1 = time.perf_counter()

            infer_ms = (t1 - t0) * 1000.0
            infer_ms_smooth = infer_ms if infer_ms_smooth == 0.0 else (0.9 * infer_ms_smooth + 0.1 * infer_ms)

            det = parse_yolov8_outputs(outputs)

            H, W = frame.shape[:2]
            boxes = []
            confs = []
            class_ids = []

            # det row format varies: either 85 (obj + 80) or 84 (80) depending on export
            cols = det.shape[1]
            for row in det:
                if cols == 85:
                    obj = float(row[4])
                    scores = row[5:]
                    cls = int(np.argmax(scores))
                    conf = obj * float(scores[cls])
                elif cols == 84:
                    scores = row[4:]
                    cls = int(np.argmax(scores))
                    conf = float(scores[cls])
                else:
                    continue

                if conf < CONF_THRESH:
                    continue

                cx, cy, bw, bh = row[0:4]
                cx *= INPUT_SIZE
                cy *= INPUT_SIZE
                bw *= INPUT_SIZE
                bh *= INPUT_SIZE

                x = int((cx - bw / 2 - dw) / ratio)
                y = int((cy - bh / 2 - dh) / ratio)
                w = int(bw / ratio)
                h = int(bh / ratio)

                x = max(0, min(W - 1, x))
                y = max(0, min(H - 1, y))
                w = max(1, min(W - x, w))
                h = max(1, min(H - y, h))

                boxes.append([x, y, w, h])
                confs.append(conf)
                class_ids.append(cls)

            # NMS
            keep = cv2.dnn.NMSBoxes(boxes, confs, CONF_THRESH, NMS_THRESH)

            if len(keep) > 0:
                for i in keep.flatten():
                    x, y, w, h = boxes[i]
                    cls = class_ids[i]
                    conf = confs[i]
                    name = COCO80[cls] if 0 <= cls < len(COCO80) else str(cls)

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, f"{name} {conf:.2f}", (x, max(0, y - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

            # FPS overlay
            t_now = time.perf_counter()
            dt = t_now - t_prev
            t_prev = t_now
            inst = (1.0 / dt) if dt > 0 else 0.0
            fps_smooth = inst if fps_smooth == 0.0 else (0.9 * fps_smooth + 0.1 * inst)

            cv2.putText(frame, f"FPS {fps_smooth:.1f}   infer {infer_ms_smooth:.1f} ms",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("Pi5 CPU YOLO (OpenCV DNN)", frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord("q")):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
