#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from picamera2 import Picamera2, MappedArray
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics
from picamera2.devices.imx500.postprocess import scale_boxes


def load_labels(path: str | None, intrinsics: NetworkIntrinsics | None):
    if path:
        p = Path(path)
        if p.is_file():
            return [ln.strip() for ln in p.read_text().splitlines() if ln.strip()]
    if intrinsics and getattr(intrinsics, "labels", None):
        return list(intrinsics.labels)
    return None


class Detection:
    def __init__(self, coords_xyxy, cls_id, score, metadata, imx500: IMX500, picam2: Picamera2):
        self.cls_id = int(cls_id)
        self.score = float(score)
        # Convert from inference tensor coords to ISP output coords
        self.box = imx500.convert_inference_coords(coords_xyxy, metadata, picam2)


def parse_detections_from_outputs(
    np_outputs,
    metadata,
    imx500: IMX500,
    picam2: Picamera2,
    threshold: float,
    bbox_normalization: bool,
    bbox_order: str,
):
    """
    Handles the common IMX500 object-detection output layout:
    boxes, scores, classes.

    Some models output boxes normalized; some need x/y swap.
    """
    if np_outputs is None or len(np_outputs) < 3:
        return []

    # Typical layout from Picamera2 IMX500 examples: [boxes, scores, classes]
    boxes = np_outputs[0][0]
    scores = np_outputs[1][0]
    classes = np_outputs[2][0]

    # Ensure shapes are sane
    if boxes is None or scores is None or classes is None:
        return []

    input_w, input_h = imx500.get_input_size()  # tensor size for the model :contentReference[oaicite:4]{index=4}

    # Boxes may be in various formats; we assume xyxy in model space after fixes below.
    # Option: some models store boxes as yxxy or similar.
    if bbox_order.lower() == "yx":
        # Swap x/y in (x0, y0, x1, y1) style arrays
        boxes = boxes.copy()
        boxes[:, [0, 1, 2, 3]] = boxes[:, [1, 0, 3, 2]]

    # If normalized, convert to pixel units of the model input
    if bbox_normalization:
        # Many examples normalize by input_h for square models; we do per-axis safely.
        boxes = boxes.copy()
        boxes[:, [0, 2]] *= float(input_w)
        boxes[:, [1, 3]] *= float(input_h)

    # If boxes are still in an unscaled format, some pipelines use scale_boxes helper
    # to map to the model input size.
    # We keep it conservative: only apply if values look tiny.
    if np.max(boxes) <= 2.0:
        boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)

    dets = []
    for b, s, c in zip(boxes, scores, classes):
        if float(s) >= threshold:
            dets.append(Detection(b, c, s, metadata, imx500, picam2))
    return dets


def draw_detections(frame_bgr, detections, labels):
    for d in detections:
        x0, y0, x1, y1 = map(int, d.box)
        cv2.rectangle(frame_bgr, (x0, y0), (x1, y1), (0, 255, 0), 2)
        if labels and 0 <= d.cls_id < len(labels):
            name = labels[d.cls_id]
        else:
            name = f"class_{d.cls_id}"
        cv2.putText(
            frame_bgr,
            f"{name} {d.score:.2f}",
            (x0, max(0, y0 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--model",
        default="/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk",
        help="Path to an IMX500 .rpk model",
    )
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--labels", default=None, help="Optional labels.txt file")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--bbox-normalization", action="store_true", help="If model outputs normalized boxes")
    ap.add_argument("--bbox-order", choices=["xy", "yx"], default="xy", help="Swap box coordinate order if needed")
    args = ap.parse_args()

    # Must load IMX500 before Picamera2 is instantiated (common pattern in examples)
    imx500 = IMX500(args.model)
    intr = imx500.network_intrinsics
    if not intr:
        intr = NetworkIntrinsics()
        intr.task = "object detection"

    labels = load_labels(args.labels, intr)

    picam2 = Picamera2(imx500.camera_num)
    config = picam2.create_preview_configuration(
        main={"format": "BGR888", "size": (args.width, args.height)},
        controls={"FrameRate": float(args.fps)},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)

    win = "IMX500 Detection (cv2)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    last_t = time.perf_counter()
    fps = 0.0

    try:
        while True:
            # Capture frame + metadata (metadata drives IMX500.get_outputs) :contentReference[oaicite:5]{index=5}
            frame = picam2.capture_array("main")
            metadata = picam2.capture_metadata()

            np_outputs = imx500.get_outputs(metadata, add_batch=True)
            dets = parse_detections_from_outputs(
                np_outputs,
                metadata,
                imx500,
                picam2,
                threshold=args.threshold,
                bbox_normalization=args.bbox_normalization,
                bbox_order=args.bbox_order,
            )

            draw_detections(frame, dets, labels)

            # FPS overlay
            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else (1.0 / dt)
            cv2.putText(
                frame,
                f"FPS: {fps:.1f}  dets:{len(dets)}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(win, frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q") or k == 27:
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()