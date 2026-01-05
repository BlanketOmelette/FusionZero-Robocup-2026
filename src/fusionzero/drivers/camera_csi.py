import os
import time
from typing import Optional

import cv2
from picamera2 import Picamera2


class CsiCamera:
    """
    CSI camera wrapper for Pi 5 dual CSI.
    Returns BGR frames for OpenCV (so colors look correct).
    """

    def __init__(self, camera_num: int, width: int = 640, height: int = 480, framerate: Optional[int] = None):
        self.camera_num = camera_num
        self.width = width
        self.height = height

        self.cam = Picamera2(camera_num=camera_num)

        cfg = self.cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},
        )
        self.cam.configure(cfg)
        self.cam.start()

        # give sensor a moment to settle
        time.sleep(0.2)

    def read(self):
        frame_rgb = self.cam.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        return frame_bgr

    def close(self):
        try:
            self.cam.stop()
        except Exception:
            pass
        try:
            self.cam.close()
        except Exception:
            pass


def list_csi_cameras():
    infos = Picamera2.global_camera_info()
    return infos


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY"))
