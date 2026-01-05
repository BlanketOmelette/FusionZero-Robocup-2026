import time
import cv2

from fusionzero.drivers.camera_csi import CsiCamera, list_csi_cameras, has_display


def main():
    infos = list_csi_cameras()
    print("Detected CSI cameras:")
    for i, info in enumerate(infos):
        print(f"  [{i}] {info}")

    if len(infos) < 2:
        print("\nNeed 2 cameras detected. Run: rpicam-hello --list-cameras")
        return

    cam0 = CsiCamera(0)
    cam1 = CsiCamera(1)

    gui = has_display()
    last = time.perf_counter()
    frames = 0

    try:
        while True:
            f0 = cam0.read()
            f1 = cam1.read()

            frames += 1
            now = time.perf_counter()
            if now - last >= 1.0:
                print(f"FPS(loop): {frames/(now-last):.1f}")
                last = now
                frames = 0

            if gui:
                cv2.imshow("CSI 0", f0)
                cv2.imshow("CSI 1", f1)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q")):
                    break
            else:
                # headless fallback
                cv2.imwrite("/tmp/csi0.jpg", f0)
                cv2.imwrite("/tmp/csi1.jpg", f1)
                time.sleep(0.2)

    finally:
        cam0.close()
        cam1.close()
        if gui:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
