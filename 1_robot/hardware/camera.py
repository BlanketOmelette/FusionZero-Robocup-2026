from core.shared_imports import cv2, np, time, threading, Thread, Picamera2, Transform
from core.utilities import debug

_PRESETS = {
    "line": {
        "index": 0,
        "size": (320, 200),
        "roi": (0, 0, 320, 200),
        "vflip": False,
        "hflip": False,
        "target_fps": 56,
    },
    "evac": {
        "index": 1,
        "size": (320, 200),
        "roi": (0, 0, 320, 200),
        "vflip": True,
        "hflip": False,
        "target_fps": 30,
    },
}


def _clamp_int(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


class Camera:
    def __init__(self, camera_type: str = "line"):
        if camera_type not in _PRESETS:
            raise ValueError(f"Unknown camera_type '{camera_type}'. Valid: {list(_PRESETS.keys())}")

        cfg = _PRESETS[camera_type]
        self.camera_type = camera_type

        self.X11 = True
        self.debug = False

        self.WIDTH, self.HEIGHT = cfg["size"]
        self.roi = cfg["roi"]
        self.FLIP_V = cfg["vflip"]
        self.FLIP_H = cfg["hflip"]
        self.TARGET_FPS = cfg["target_fps"]

        self.color_format = "RGB888"
        self._init_debug_polygons()

        self.camera = Picamera2(camera_num=cfg["index"])

        transform = Transform(vflip=self.FLIP_V, hflip=self.FLIP_H)
        camera_config = self.camera.create_video_configuration(
            main={"size": (self.WIDTH, self.HEIGHT), "format": self.color_format},
            transform=transform,
        )
        self.camera.configure(camera_config)

        try:
            self.camera.set_controls({"FrameRate": float(self.TARGET_FPS)})
        except Exception:
            pass

        # Threaded latest frame
        self._latest = None
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame_id = 0


        # FPS tracking
        self.fps = 0.0
        self._t_last = time.perf_counter()
        self._fps_alpha = 0.15

        debug(["INITIALISATION", f"CAMERA({camera_type})", "✓"], [25, 25, 50])

    def _init_debug_polygons(self):
        w = int(self.WIDTH)
        h = int(self.HEIGHT)

        self.light_point_left = np.array([[120, 70], [40, 70], [45, 100], [125, 100]], dtype=np.float32)
        self.light_point_right = np.array([[290, 45], [195, 45], [195, 85], [280, 85]], dtype=np.float32)
        self.light_point_mid_top = np.array([[w - 40, 100], [50, 110], [65, 130], [w - 45, 120]], dtype=np.float32)
        self.light_point_mid = np.array([[w - 30, 125], [70, 140], [80, 160], [w - 40, 145]], dtype=np.float32)
        self.dark_left = np.array([[0, h - 70], [0, h], [70, h]], dtype=np.float32)
        self.dark_right = np.array([[w, h - 70], [w, h], [w - 70, h]], dtype=np.float32)

    def _apply_roi(self, frame):
        if frame is None or self.roi is None:
            return frame

        h, w = frame.shape[:2]
        x0, y0, rw, rh = self.roi

        x0 = _clamp_int(int(x0), 0, w)
        y0 = _clamp_int(int(y0), 0, h)
        x1 = _clamp_int(int(x0 + rw), 0, w)
        y1 = _clamp_int(int(y0 + rh), 0, h)

        if x1 <= x0 or y1 <= y0:
            return frame
        return frame[y0:y1, x0:x1]

    def _draw_debug(self, frame_rgb):
        if not (self.X11 and self.debug):
            return frame_rgb

        img = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        cv2.polylines(img, [np.int32(self.light_point_left)], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.polylines(img, [np.int32(self.light_point_right)], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.polylines(img, [np.int32(self.light_point_mid_top)], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.polylines(img, [np.int32(self.light_point_mid)], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.polylines(img, [np.int32(self.dark_left)], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.polylines(img, [np.int32(self.dark_right)], isClosed=True, color=(0, 255, 0), thickness=2)

        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def _worker(self):
        while self._running:
            try:
                frame = self.camera.capture_array("main")
            except Exception:
                time.sleep(0.005)
                continue

            frame = self._apply_roi(frame)
            frame = self._draw_debug(frame)

            now = time.perf_counter()

            with self._cond:
                self._latest = frame
                self._frame_id += 1
                self._cond.notify()

            # FPS tracking
            dt = now - self._t_last
            self._t_last = now
            if dt > 1e-6:
                inst = 1.0 / dt
                if self.fps <= 0.0:
                    self.fps = inst
                else:
                    self.fps = (1.0 - self._fps_alpha) * self.fps + self._fps_alpha * inst


    def capture_array(self):
        """
        Blocking call.
        Returns the next available frame exactly once.
        """
        with self._cond:
            last_id = self._frame_id
            while self._running and self._frame_id == last_id:
                self._cond.wait()

            return self._latest


    def start(self):
        if self._running:
            return
        self.camera.start()
        self._running = True
        self._t_last = time.perf_counter()
        self._thread = Thread(target=self._worker, daemon=True)
        self._thread.start()

        t0 = time.perf_counter()
        while self._latest is None and (time.perf_counter() - t0) < 1.0:
            time.sleep(0.001)


    def stop(self):
        if not self._running:
            return
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)
            self._thread = None
        try:
            self.camera.stop()
        except Exception:
            pass

    def close(self):
        self.stop()
        if self.camera:
            try:
                self.camera.close()
            except Exception:
                pass
            self.camera = None
            debug(["TERMINATION", f"CAMERA({self.camera_type})", "✓"], [25, 25, 50])
        else:
            debug(["TERMINATION", f"CAMERA({self.camera_type})", "X"], [25, 25, 50])

        if self.X11:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass