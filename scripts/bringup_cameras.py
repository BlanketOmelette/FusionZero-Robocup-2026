#!/usr/bin/env python3
import time
from fusionzero.drivers.camera import AsyncCamera, list_cameras, has_display

SHOW_WINDOWS = True

def main():
    print("Cameras:")
    for i, info in enumerate(list_cameras()):
        print(f"  {i}: {info}")

    gui = bool(SHOW_WINDOWS and has_display())
    if gui:
        import cv2
        cv2.namedWindow("Wide", cv2.WINDOW_NORMAL)
        cv2.namedWindow("AI", cv2.WINDOW_NORMAL)

    wide = AsyncCamera(0)  # uses defaults from camera.py
    ai = AsyncCamera(1)

    try:
        while True:
            fw = wide.read()
            fa = ai.read()

            if gui and fw is not None and fa is not None:
                import cv2
                cv2.imshow("Wide", cv2.cvtColor(fw, cv2.COLOR_RGB2BGR))
                cv2.imshow("AI", cv2.cvtColor(fa, cv2.COLOR_RGB2BGR))
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            else:
                print(f"FPS wide={wide.fps():5.1f} ai={ai.fps():5.1f}")
                time.sleep(1.0)
    finally:
        wide.close()
        ai.close()
        if gui:
            import cv2
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
