from __future__ import annotations

import os
import time
import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple, List


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY"))


def list_cameras() -> List[Dict[str, Any]]:
    try:
        from picamera2 import Picamera2
    except Exception as e:
        raise ImportError("Picamera2 missing. Try: sudo apt install -y python3-picamera2") from e
    return list(Picamera2.global_camera_info())


@dataclass(frozen=True)
class CameraConfig:
    # What you want to process
    out_size: Tuple[int, int] = (320, 200)
    out_format: str = "RGB888"  # OpenCV friendly, no cvtColor needed

    # If None, we pick defaults based on sensor model
    sensor_size: Optional[Tuple[int, int]] = None
    fps: Optional[float] = None

    # If True, lock fps with FrameDurationLimits
    lock_fps: bool = True

    # Low latency
    buffer_count: int = 4
    queue: bool = False  # False avoids frame queue latency

    # Orientation
    auto_orientation: bool = True
    hflip: Optional[bool] = None
    vflip: Optional[bool] = None

    # Exposure defaults: leave auto on
    ae_enable: bool = True
    awb_enable: bool = True
    exposure_us: Optional[int] = None
    analogue_gain: Optional[float] = None

    # Start up settle
    settle_s: float = 0.12


class Camera:
    """
    Plug and play threaded camera.

    Usage:
        cam = Camera(0)
        frame = cam.read()   # latest frame or None until first frame arrives
        print(cam.fps)

    Defaults:
        imx708_wide: sensor 2304x1296, ~56 fps, wide view
        imx500:      sensor 2028x1520, 30 fps
    """

    def __init__(self, index: int, cfg: Optional[CameraConfig] = None):
        try:
            from picamera2 import Picamera2
            from libcamera import Transform
        except Exception as e:
            raise ImportError("Camera stack missing. Try: sudo apt install -y python3-picamera2") from e

        self.index = int(index)
        self.picam = Picamera2(camera_num=self.index)

        info = self._get_global_info()
        model = str(info.get("Model", "")).lower()
        rotation = int(info.get("Rotation", 0) or 0)

        if cfg is None:
            cfg = CameraConfig()
        cfg = self._apply_model_defaults(cfg, model)

        # Decide flips
        hflip, vflip = self._resolve_flips(cfg, rotation)

        transform = Transform(hflip=bool(hflip), vflip=bool(vflip))

        # Build configuration
        main = {"size": cfg.out_size, "format": cfg.out_format}

        # Force sensor mode to keep wide view / expected fps
        sensor = None
        if cfg.sensor_size is not None:
            sensor = {"output_size": cfg.sensor_size}

        # Configure with preferred format, fallback if needed
        configured = False
        last_err: Optional[Exception] = None

        for fmt_try in (cfg.out_format, "RGB888"):
            try:
                main_try = {"size": cfg.out_size, "format": fmt_try}
                kwargs: Dict[str, Any] = dict(
                    main=main_try,
                    transform=transform,
                    buffer_count=int(cfg.buffer_count),
                    queue=bool(cfg.queue),
                )
                if sensor is not None:
                    kwargs["sensor"] = sensor

                video_cfg = self.picam.create_video_configuration(**kwargs)
                self.picam.configure(video_cfg)
                self._active_format = fmt_try
                configured = True
                break
            except Exception as e:
                last_err = e

        if not configured:
            raise RuntimeError(f"Failed to configure camera {self.index}: {last_err}")

        # Controls
        controls: Dict[str, Any] = {
            "AeEnable": bool(cfg.ae_enable),
            "AwbEnable": bool(cfg.awb_enable),
        }

        if cfg.fps is not None and cfg.lock_fps:
            frame_us = max(1, int(round(1_000_000 / float(cfg.fps))))
            controls["FrameDurationLimits"] = (frame_us, frame_us)

        if cfg.exposure_us is not None:
            controls["AeEnable"] = False
            controls["ExposureTime"] = int(cfg.exposure_us)
            if cfg.analogue_gain is not None:
                controls["AnalogueGain"] = float(cfg.analogue_gain)

        try:
            self.picam.set_controls(controls)
        except Exception:
            # Not all controls are supported on all pipelines
            pass

        self.picam.start()
        time.sleep(float(cfg.settle_s))

        # Thread state
        self._lock = threading.Lock()
        self._frame = None
        self._frame_t = 0.0

        self._running = True
        self._fps = 0.0
        self._fps_count = 0
        self._fps_t0 = time.perf_counter()

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        self.cfg = cfg
        self.model = model
        self.rotation = rotation

    def _get_global_info(self) -> Dict[str, Any]:
        infos = list_cameras()
        if 0 <= self.index < len(infos):
            return infos[self.index]
        return {}

    def _apply_model_defaults(self, cfg: CameraConfig, model: str) -> CameraConfig:
        # Respect user overrides first
        sensor_size = cfg.sensor_size
        fps = cfg.fps

        if "imx708" in model:
            # Wide view, fastest available: 2304x1296 ~56fps (your list shows (0,0)/4608x2592 crop)
            if sensor_size is None:
                sensor_size = (2304, 1296)
            if fps is None:
                fps = 56.0

        elif "imx500" in model:
            # AI camera limit: 2028x1520 30fps
            if sensor_size is None:
                sensor_size = (2028, 1520)
            if fps is None:
                fps = 30.0

        else:
            if fps is None:
                fps = 30.0

        return CameraConfig(
            **{
                **cfg.__dict__,
                "sensor_size": sensor_size,
                "fps": fps,
            }
        )

    def _resolve_flips(self, cfg: CameraConfig, rotation: int) -> Tuple[bool, bool]:
        # If user explicitly set flips, use them
        if cfg.hflip is not None or cfg.vflip is not None:
            return bool(cfg.hflip), bool(cfg.vflip)

        if not cfg.auto_orientation:
            return False, False

        # Use Rotation metadata as a sensible default
        # 180 means upside down, so flip both
        if rotation == 180:
            return True, True
        if rotation == 90:
            # Rotation isn't a flip; we leave it alone here
            return False, False
        if rotation == 270:
            return False, False
        return False, False

    def _loop(self):
        while self._running:
            try:
                img = self.picam.capture_array("main")
            except Exception:
                time.sleep(0.005)
                continue

            now = time.perf_counter()
            with self._lock:
                self._frame = img
                self._frame_t = now

            self._fps_count += 1
            dt = now - self._fps_t0
            if dt >= 1.0:
                self._fps = self._fps_count / dt
                self._fps_count = 0
                self._fps_t0 = now

    def read(self):
        # Latest frame; returns None until first frame arrives
        with self._lock:
            if self._frame is None:
                return None
            # Copy so callers never race the writer thread
            return self._frame.copy()

    @property
    def fps(self) -> float:
        return float(self._fps)

    def age_s(self) -> float:
        with self._lock:
            t = self._frame_t
        if t == 0.0:
            return 1e9
        return time.perf_counter() - t

    def close(self):
        self._running = False
        try:
            self._thread.join(timeout=0.5)
        except Exception:
            pass
        try:
            self.picam.stop()
        except Exception:
            pass
        try:
            self.picam.close()
        except Exception:
            pass
