#!/usr/bin/env python3
import os, sys
import time
from datetime import datetime
from pathlib import Path
import argparse

import cv2
import numpy as np
from PIL import Image

# Make imports work when run as: python3 1_robot/tools/xxx.py
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from core.utilities import start_display, stop_display, show
from hardware.camera import Camera
from hardware.touch import Touch   # <-- change this import if your Touch class is elsewhere


def stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def save_rgb(rgb: np.ndarray, path: Path, quality: int = 92) -> None:
    Image.fromarray(rgb).save(str(path), quality=quality, optimize=True)


def pad_to_square(frame: np.ndarray, value=(0, 0, 0)) -> np.ndarray:
    """Pad to square (NO cropping)."""
    h, w = frame.shape[:2]
    if h == w:
        return frame
    if w > h:
        pad = w - h
        top = pad // 2
        bottom = pad - top
        return cv2.copyMakeBorder(frame, top, bottom, 0, 0, cv2.BORDER_CONSTANT, value=value)
    else:
        pad = h - w
        left = pad // 2
        right = pad - left
        return cv2.copyMakeBorder(frame, 0, 0, left, right, cv2.BORDER_CONSTANT, value=value)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/ball_det_dataset/raw", help="output root folder")
    ap.add_argument("--cooldown", type=float, default=0.20, help="seconds between saves while held")
    ap.add_argument("--jpeg_quality", type=int, default=92)

    ap.add_argument("--preview", action="store_true", help="preview using your show()")
    ap.add_argument("--opencv_preview", action="store_true", help="preview using cv2.imshow")

    ap.add_argument("--mode", choices=["raw", "square"], default="raw",
                    help="raw saves 320x200 as-is; square pads (no crop) then resizes")
    ap.add_argument("--square_size", type=int, default=320, help="only for mode=square (320 or 640)")

    ap.add_argument("--hflip", action="store_true", help="extra horizontal flip AFTER capture")
    ap.add_argument("--vflip", action="store_true", help="extra vertical flip AFTER capture")

    ap.add_argument("--pressed_value", type=int, default=1,
                    help="value that means pressed (your robot: 1)")
    ap.add_argument("--crop_y", type=int, default=70,
                help="Keep only rows y >= crop_y (crop off the top). Use 0 to disable.")

    args = ap.parse_args()

    session = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_dir = Path(args.out) / session
    out_dir.mkdir(parents=True, exist_ok=True)

    # Use your existing evac camera pipeline
    cam = Camera("evac")
    cam.start()

    # Use your Touch class (Back Left is index 2)
    touch = Touch()

    if args.preview:
        start_display()

    print(f"[INFO] Saving to: {out_dir.resolve()}")
    print("[INFO] Hold Back Left switch to capture. Ctrl+C to quit.")
    print(f"[INFO] pressed_value={args.pressed_value} mode={args.mode} square_size={args.square_size}")
    print(f"[INFO] extra flips: hflip={args.hflip} vflip={args.vflip}")

    last_save = 0.0
    saved = 0

    try:
        while True:
            frame = cam.capture_array()  # RGB, from your evac preset (usually 320x200)
            # Crop off top region: keep y >= crop_y
            if args.crop_y > 0:
                y0 = min(args.crop_y, frame.shape[0])
                frame = frame[y0:, :]


            if frame is None:
                time.sleep(0.005)
                continue

            # Optional extra flips (use only if camera preset is not flipped how you want)
            if args.hflip:
                frame = cv2.flip(frame, 1)
            if args.vflip:
                frame = cv2.flip(frame, 0)

            out_frame = frame

            # If you want 320x320 without cropping, pad then resize
            if args.mode == "square":
                out_frame = pad_to_square(out_frame)
                if out_frame.shape[0] != args.square_size:
                    out_frame = cv2.resize(out_frame, (args.square_size, args.square_size),
                                           interpolation=cv2.INTER_AREA)

            if args.preview:
                show(out_frame, name="evac_cap", display=True)

            if args.opencv_preview:
                cv2.imshow("evac_cap", out_frame[:, :, ::-1])  # RGB -> BGR
                if (cv2.waitKey(1) & 0xFF) == 27:  # ESC
                    break

            # Back Left is index 2
            bl = touch.read()[2]
            pressed = (int(bl) == int(args.pressed_value))

            now = time.monotonic()
            if pressed and (now - last_save) >= args.cooldown:
                fname = f"{stamp()}_BL.jpg"
                save_rgb(out_frame, out_dir / fname, quality=args.jpeg_quality)
                saved += 1
                last_save = now
                print(f"[SAVE] {saved:05d} {fname}")

            time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            cam.close()
        except Exception:
            pass
        try:
            touch.close()
        except Exception:
            pass
        if args.preview:
            stop_display()
        if args.opencv_preview:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

    print("[DONE] Total saved:", saved)


if __name__ == "__main__":
    main()
