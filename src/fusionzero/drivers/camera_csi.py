import time

class CsiCamera:
    def __init__(self, index: int, width: int = 640, height: int = 480):
        try:
            from picamera2 import Picamera2
        except Exception as e:
            raise ImportError(
                "Picamera2 missing. Install with: sudo apt install -y python3-picamera2"
            ) from e

        self.picam = Picamera2(camera_num=index)
        cfg = self.picam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam.configure(cfg)
        self.picam.start()
        time.sleep(0.2)

    def read(self):
        return self.picam.capture_array()

    def close(self):
        try:
            self.picam.stop()
        except Exception:
            pass
        try:
            self.picam.close()
        except Exception:
            pass
