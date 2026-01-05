#!/usr/bin/env python3
import time

from fusionzero.drivers.camera import Camera, has_display, list_cameras

SHOW_WINDOWS = True

def main():
    print("Cameras:")
    for i, info in enumerate(list_cameras()):
        print(f"  {i}: {info}")

    gui = bool(SHOW_WINDOWS and has_display())
    if gui:
        import cv2
        cv2.namedWindow("wide", cv2.WINDOW_NORMAL)
        cv2.namedWindow("ai", cv2.WINDOW_NORMAL)
    else:
        print("Headless mode, printing FPS")

    wide = Camera(0)
    ai = Camera(1)

    last = time.perf_counter()
    try:
        while True:
            f0 = wide.read()
            f1 = ai.read()

            now = time.perf_counter()
            if now - last >= 1.0:
                print(f"FPS wide={wide.fps:5.1f} ai={ai.fps:5.1f} age_w={wide.age_s():.3f}s age_a={ai.age_s():.3f}s")
                last = now

            if gui:
                import cv2
                if f0 is not None:
                    cv2.imshow("wide", f0)
                if f1 is not None:
                    cv2.imshow("ai", f1)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            else:
                time.sleep(0.01)

    finally:
        wide.close()
        ai.close()
        if gui:
            import cv2
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
