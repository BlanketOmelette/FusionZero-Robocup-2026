#!/usr/bin/env python3
from __future__ import annotations

import os
import time
from datetime import datetime
import argparse

from PIL import Image

from hardware.robot import line_camera, touch


def stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def save_rgb(rgb, path: str) -> None:
    Image.fromarray(rgb).save(path, quality=95)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/silver_dataset", help="output folder")
    ap.add_argument("--label", choices=["silver", "no_silver", "unlabeled"], default="unlabeled",
                    help="folder to save into for this run")
    ap.add_argument("--burst", type=int, default=3, help="images saved per press")
    ap.add_argument("--cooldown", type=float, default=0.04, help="seconds between burst frames")
    ap.add_argument("--debounce", type=float, default=0.5, help="min seconds between captures while held")
    ap.add_argument("--hold_release", action="store_true",
                    help="if set, only trigger again after BL is released and pressed again")
    args = ap.parse_args()

    out_dir = os.path.join(args.out, args.label)
    os.makedirs(out_dir, exist_ok=True)

    # Try start camera if your wrapper supports it
    try:
        line_camera.start()
    except Exception:
        pass

    print("Capture on Back-Left touch (BL)")
    print("Touch indices: [FL, FR, BL, BR] -> BL is index 2")
    print("Saving to:", out_dir)
    print("Press BL to capture. Ctrl+C to stop.")

    last_capture_t = 0.0
    waiting_release = False
    saved = 0

    try:
        while True:
            vals = touch.read()  # [FL, FR, BL, BR] released=1 pressed=0
            bl_pressed = (vals[2] == 0)

            now = time.time()

            if args.hold_release:
                if waiting_release:
                    if not bl_pressed:
                        waiting_release = False
                    time.sleep(0.01)
                    continue

                if bl_pressed:
                    # capture
                    for _ in range(args.burst):
                        frame = line_camera.capture_array()
                        if frame is None:
                            time.sleep(0.01)
                            continue
                        name = f"{stamp()}_BL.jpg"
                        save_rgb(frame, os.path.join(out_dir, name))
                        saved += 1
                        time.sleep(args.cooldown)

                    print(f"Captured {args.burst} -> {args.label} | total {saved}")
                    waiting_release = True
                    time.sleep(0.05)
                    continue

            else:
                # time-based debounce (allows re-trigger if you keep holding long enough)
                if bl_pressed and (now - last_capture_t >= args.debounce):
                    last_capture_t = now

                    for _ in range(args.burst):
                        frame = line_camera.capture_array()
                        if frame is None:
                            time.sleep(0.01)
                            continue
                        name = f"{stamp()}_BL.jpg"
                        save_rgb(frame, os.path.join(out_dir, name))
                        saved += 1
                        time.sleep(args.cooldown)

                    print(f"Captured {args.burst} -> {args.label} | total {saved}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            line_camera.stop()
        except Exception:
            pass

    print("Done. Total saved:", saved)


if __name__ == "__main__":
    main()
