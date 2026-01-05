import os
import time
from dataclasses import dataclass
import threading
from typing import Any, Dict, List, Optional, Tuple


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY"))


def list_cameras() -> List[Dict[str, Any]]:
    try:
        from picamera2 import Picamera2
    except Exception as e:
        raise ImportError(
            "Picamera2 missing. Install with: sudo apt install -y python3-picamera2"
        ) from e
    return list(Picamera2.global_camera_info())


@dataclass(frozen=True)
class CameraConfig:
    # Output to your vision code
    width: int = 320
    height: int = 200
    main_format: str = "RGB888"

    # If fps is None, we auto select based on camera model:
    # - imx708_wide -> 120 fps + sensor mode 1536x864
    # - imx500      -> 30 fps (AI cam limit)
    fps: Optional[int] = None

    # Orientation
    hflip: bool = False
    vflip: bool = False

    # Low latency
    buffer_count: int = 2
    queue: bool = False

    # RAW (off by default for latency)
    include_raw: bool = False
    raw_size: Tuple[int, int] = (2304, 1296)
    raw_format: str = "SBGGR10"

    # Optional forced sensor mode (if None, driver may auto set it)
    sensor_output_size: Optional[Tuple[int, int]] = None
    sensor_bit_depth: int = 10

    # Exposure controls
    # If exposure_us is set, AeEnable will be disabled
    ae_enable: bool = True
    awb_enable: bool = True
    exposure_us: Optional[int] = None
    analogue_gain: Optional[float] = None

    # Auto defaults based on sensor model
    auto_defaults: bool = True

    # Startup settle
    settle_s: float = 0.12


class Camera:
    """
    Picamera2 CSI camera wrapper with sensor-aware defaults.

    Default behaviour (no config passed):
    - IMX708 wide: 120 fps using sensor mode 1536x864, output scaled to 320x200
    - IMX500 AI cam: 30 fps, output scaled to 320x200
    """

    def __init__(self, index: int, cfg: CameraConfig = CameraConfig()):
        try:
            from picamera2 import Picamera2
            from libcamera import Transform
        except Exception as e:
            raise ImportError(
                "Camera stack missing. Install with: sudo apt install -y python3-picamera2"
            ) from e

        self.index = int(index)
        self.picam = Picamera2(camera_num=self.index)

        model = self._detect_model()
        cfg = self._apply_auto_defaults(cfg, model)
        self.cfg = cfg

        transform = Transform(hflip=bool(cfg.hflip), vflip=bool(cfg.vflip))

        main = {"size": (cfg.width, cfg.height), "format": cfg.main_format}
        raw = {"size": cfg.raw_size, "format": cfg.raw_format} if cfg.include_raw else None

        fps = max(1, int(cfg.fps if cfg.fps is not None else 30))
        frame_us = max(1, int(1_000_000 / fps))

        controls: Dict[str, Any] = {
            "FrameDurationLimits": (frame_us, frame_us),
            "FrameRate": float(fps),  # ignored on some stacks, harmless otherwise
        }

        # Exposure logic
        if cfg.exposure_us is not None:
            controls["AeEnable"] = False
            controls["ExposureTime"] = int(cfg.exposure_us)
            if cfg.analogue_gain is not None:
                controls["AnalogueGain"] = float(cfg.analogue_gain)
        else:
            controls["AeEnable"] = bool(cfg.ae_enable)

        controls["AwbEnable"] = bool(cfg.awb_enable)

        sensor = None
        if cfg.sensor_output_size is not None:
            sensor = {"output_size": cfg.sensor_output_size, "bit_depth": int(cfg.sensor_bit_depth)}

        configured = False
        last_err: Optional[Exception] = None

        # Try combinations: raw+sensor, no-raw+sensor, raw+no-sensor, no-raw+no-sensor
        attempts = [(True, True), (False, True), (True, False), (False, False)]
        for use_raw, use_sensor in attempts:
            try:
                kwargs: Dict[str, Any] = dict(
                    main=main,
                    transform=transform,
                    buffer_count=int(cfg.buffer_count),
                    queue=bool(cfg.queue),
                    controls=controls,
                )
                if use_raw and raw is not None:
                    kwargs["raw"] = raw
                if use_sensor and sensor is not None:
                    kwargs["sensor"] = sensor

                video_cfg = self.picam.create_video_configuration(**kwargs)
                self.picam.configure(video_cfg)
                configured = True
                break
            except Exception as e:
                last_err = e

        if not configured:
            raise RuntimeError(f"Failed to configure camera index {self.index} ({model}). Last error: {last_err}")

        self.picam.start()
        time.sleep(float(cfg.settle_s))

        try:
            self.picam.set_controls(controls)
        except Exception:
            pass

    def _detect_model(self) -> str:
        # Best effort: pull model name from global info
        try:
            infos = list_cameras()
            info = infos[self.index] if self.index < len(infos) else {}
            for key in ("Model", "model", "Name", "name", "Id", "id"):
                val = info.get(key)
                if isinstance(val, str) and val:
                    return val
            return str(info)
        except Exception:
            return "unknown"

    def _apply_auto_defaults(self, cfg: CameraConfig, model: str) -> CameraConfig:
        if not cfg.auto_defaults:
            # If user disables auto, still ensure fps not None
            return cfg if cfg.fps is not None else CameraConfig(**{**cfg.__dict__, "fps": 30})

        m = model.lower()

        # Decide FPS if unspecified
        fps = cfg.fps
        sensor_output_size = cfg.sensor_output_size
        include_raw = cfg.include_raw
        ae_enable = cfg.ae_enable
        exposure_us = cfg.exposure_us

        if fps is None:
            if "imx708" in m:
                fps = 56
            elif "imx500" in m:
                fps = 30
            else:
                fps = 30

        # If we are aiming for 120 on IMX708, force the 120 fps sensor mode unless user set one
        if "imx708" in m and fps >= 100 and sensor_output_size is None:
            sensor_output_size = (2304, 1296)

        # On IMX500, don’t bother trying RAW by default (latency, no benefit for your pipeline)
        if "imx500" in m:
            include_raw = False

        # Keep 120 fps stable: if user didn’t specify exposure, force a short one
        # (Otherwise AE can lengthen exposure and effectively reduce fps)
        if "imx708" in m and fps >= 100 and exposure_us is None:
            exposure_us = None
            ae_enable = True

        return CameraConfig(
            **{
                **cfg.__dict__,
                "fps": fps,
                "sensor_output_size": sensor_output_size,
                "include_raw": include_raw,
                "ae_enable": ae_enable,
                "exposure_us": exposure_us,
            }
        )

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

import threading
from typing import Optional, Tuple

class AsyncCamera:
    """
    Background grabbing wrapper.
    - Starts its own thread
    - Always stores the latest frame
    - read() is non blocking (returns latest or None until first frame arrives)
    """
    def __init__(self, index: int, cfg: CameraConfig = CameraConfig(), copy_frames: bool = False):
        self._cam = Camera(index, cfg)
        self._copy = bool(copy_frames)

        self._lock = threading.Lock()
        self._frame = None
        self._running = True

        self._frames = 0
        self._fps = 0.0
        self._t0 = time.perf_counter()

        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        while self._running:
            img = self._cam.read()
            if self._copy:
                img = img.copy()

            now = time.perf_counter()
            with self._lock:
                self._frame = img

            self._frames += 1
            dt = now - self._t0
            if dt >= 1.0:
                self._fps = self._frames / dt
                self._frames = 0
                self._t0 = now

    def read(self):
        with self._lock:
            return self._frame

    def fps(self) -> float:
        return float(self._fps)

    def close(self):
        self._running = False
        self._th.join(timeout=1.0)
        self._cam.close()
