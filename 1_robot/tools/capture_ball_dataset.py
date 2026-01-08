#!/usr/bin/env python3
import argparse
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image

from picamera2 import Picamera2

# Project hardware
from hardware.robot import touch


def is_pressed(v: int) -> bool:
    return v == 1


def center_crop_square(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    s = min(h, w)
    y0 = (h - s) // 2
    x0 = (w - s) // 2
    return img[y0 : y0 + s, x0 : x0 + s]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/ball_det_dataset/raw", help="Output root folder")
    ap.add_argument("--cam_w", type=int, default=640)
    ap.add_argument("--cam_h", type=int, default=480)
    ap.add_argument("--save_size", type=int, default=640, help="Saved image size (square). Use 640 or 320.")
    ap.add_argument("--cooldown", type=float, default=0.20, help="Seconds between saves while held")
    ap.add_argument("--no_preview", action="store_true", help="Disable preview window")
    ap.add_argument("--jpeg_quality", type=int, default=92)
    args = ap.parse_args()

    # Session folder
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_dir = Path(args.out) / stamp
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Saving to: {out_dir.resolve()}")
    print("[INFO] Hold Back Left touch to capture. Ctrl+C to quit.")

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (args.cam_w, args.cam_h), "format": "RGB888"},
        buffer_count=4,
    )
    picam2.configure(config)
    picam2.start()

    last_save = 0.0
    saved = 0

    try:
        while True:
            frame = picam2.capture_array("main")  # RGB888

            # Make dataset square to better match 320x320 style deploy
            frame = center_crop_square(frame)

            # Optional resize (keeps labels simpler and export consistent)
            if frame.shape[0] != args.save_size:
                frame = np.array(Image.fromarray(frame).resize((args.save_size, args.save_size)))

            # Preview (simple OpenCV-less preview via PIL is too slow, so keep it minimal)
            if not args.no_preview:
                # Lightweight preview using Picamera2 preview is fine, but many setups are headless.
                # If you want OpenCV preview in your environment, swap this for your core.utilities.show().
                pass

            # Read touch sensors: [FL, FR, BL, BR]
            vals = touch.read()
            bl = vals[2]

            now = time.monotonic()
            if is_pressed(bl) and (now - last_save) >= args.cooldown:
                fname = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jpg"
                fpath = out_dir / fname

                Image.fromarray(frame).save(
                    fpath,
                    format="JPEG",
                    quality=args.jpeg_quality,
                    optimize=True,
                )

                saved += 1
                last_save = now
                print(f"[SAVE] {saved:05d} {fname}")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
    finally:
        picam2.stop()


if __name__ == "__main__":
    main()
