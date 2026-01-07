import os, sys
import time
from datetime import datetime
import argparse
from PIL import Image

# Make imports work when run as: python3 1_robot/tools/xxx.py
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from core.utilities import start_display, stop_display, show
from hardware.robot import line_camera, touch, led


def stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def save_rgb(rgb, path: str) -> None:
    Image.fromarray(rgb).save(path, quality=95)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/silver_dataset", help="output folder")
    ap.add_argument("--label", choices=["silver", "no_silver", "unlabeled"], default="unlabeled")
    ap.add_argument("--preview", action="store_true", help="show live line camera window")
    ap.add_argument("--interval", type=float, default=0.25, help="seconds between captures while holding BL")
    ap.add_argument("--burst", type=int, default=1, help="images per capture event (usually keep 1)")
    ap.add_argument("--cooldown", type=float, default=0.0, help="seconds between burst frames")
    ap.add_argument("--mode", choices=["hold", "press_release"], default="hold",
                    help="hold: capture repeatedly while held. press_release: one capture then must release.")
    args = ap.parse_args()

    out_dir = os.path.join(args.out, args.label)
    os.makedirs(out_dir, exist_ok=True)

    if args.preview:
        start_display()

    try:
        line_camera.start()
    except Exception:
        pass

    led.on()

    print("Capture using Back Left touch (BL)")
    print("Touch indices: [FL, FR, BL, BR] -> BL index 2")
    print("Pressed is 0, released is 1")
    print("Mode:", args.mode)
    print("Saving to:", out_dir)
    print("Hold BL to capture. Ctrl+C to stop.")

    saved = 0
    last_cap_t = 0.0
    waiting_release = False

    try:
        while True:
            frame = line_camera.capture_array()
            if args.preview and frame is not None:
                show(frame, name="line", display=True)

            vals = touch.read()               # [FL, FR, BL, BR]
            bl_pressed = (vals[2] == 1)       # pressed is 1

            now = time.time()

            if args.mode == "press_release":
                if waiting_release:
                    if not bl_pressed:
                        waiting_release = False
                    time.sleep(0.01)
                    continue

                if bl_pressed:
                    # one capture event
                    for _ in range(args.burst):
                        fr = line_camera.capture_array()
                        if fr is None:
                            time.sleep(0.01)
                            continue
                        name = f"{stamp()}_BL.jpg"
                        save_rgb(fr, os.path.join(out_dir, name))
                        saved += 1
                        if args.cooldown > 0:
                            time.sleep(args.cooldown)

                    print(f"Captured {args.burst} -> {args.label} | total {saved}")
                    waiting_release = True
                    time.sleep(0.05)
                    continue

            else:
                # hold mode: capture repeatedly while held, rate limited by interval
                if bl_pressed and (now - last_cap_t >= args.interval):
                    last_cap_t = now

                    for _ in range(args.burst):
                        fr = line_camera.capture_array()
                        if fr is None:
                            time.sleep(0.01)
                            continue
                        name = f"{stamp()}_BL.jpg"
                        save_rgb(fr, os.path.join(out_dir, name))
                        saved += 1
                        if args.cooldown > 0:
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
        if args.preview:
            stop_display()

    print("Done. Total saved:", saved)


if __name__ == "__main__":
    main()
