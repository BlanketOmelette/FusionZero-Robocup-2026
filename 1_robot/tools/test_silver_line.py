import time
import os, sys
import json
import cv2
import numpy as np
import torch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from core.utilities import start_display, stop_display, show
from hardware.robot import line_camera, led

MODEL = "1_robot/models/silver_detector.ts"
META  = "1_robot/models/silver_detector_meta.json"

# If your camera frames are BGR (common with OpenCV), set this True.
# If your camera frames are already RGB, leave False.
FRAME_IS_BGR = False

def led_on():
    try:
        if hasattr(led, "on"): led.on()
        elif hasattr(led, "value"): led.value = 1
        elif hasattr(led, "set"): led.set(True)
        elif callable(led): led(True)
    except Exception:
        pass

def led_off():
    try:
        if hasattr(led, "off"): led.off()
        elif hasattr(led, "value"): led.value = 0
        elif hasattr(led, "set"): led.set(False)
        elif callable(led): led(False)
    except Exception:
        pass

def safe_show(win, img):
    show(img, win)

def load_meta():
    if os.path.exists(META):
        with open(META, "r") as f:
            return json.load(f)
    return {
        "img_size": 64,
        "mean": [0.485, 0.456, 0.406],
        "std":  [0.229, 0.224, 0.225],
        "crop_mode": "none",
        "recommended_threshold": 0.98,
        "recommended_streak": 3,
    }

meta = load_meta()
IMG = int(meta.get("img_size", 64))

MEAN = np.array(meta.get("mean", [0.485, 0.456, 0.406]), dtype=np.float32).reshape(1, 1, 3)
STD  = np.array(meta.get("std",  [0.229, 0.224, 0.225]), dtype=np.float32).reshape(1, 1, 3)
CROP_MODE = meta.get("crop_mode", meta.get("crop", "none"))

def crop_roi(rgb: np.ndarray) -> np.ndarray:
    h, w = rgb.shape[:2]
    if CROP_MODE == "none":
        return rgb
    if CROP_MODE == "bottom_half":
        return rgb[h // 2 :, :]
    if CROP_MODE == "bottom_band_center":
        # must match your notebook defaults if you used this crop
        y0, x0, x1 = 0.60, 0.15, 0.85
        return rgb[int(h*y0):, int(w*x0):int(w*x1)]
    return rgb

def ensure_rgb(frame: np.ndarray) -> np.ndarray:
    if FRAME_IS_BGR:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return frame

def prep(frame: np.ndarray) -> torch.Tensor:
    rgb = ensure_rgb(frame)
    roi = crop_roi(rgb)
    img = cv2.resize(roi, (IMG, IMG), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0
    img = (img - MEAN) / STD
    x = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).contiguous()
    return x

def main():
    if not os.path.exists(MODEL):
        raise FileNotFoundError(MODEL)

    # Threads can help on Pi CPU
    try:
        torch.set_num_threads(4)
        torch.set_num_interop_threads(1)
    except Exception:
        pass

    model = torch.jit.load(MODEL, map_location="cpu").eval()
    if hasattr(torch.jit, "optimize_for_inference"):
        try:
            model = torch.jit.optimize_for_inference(model)
        except Exception:
            pass

    print("IMG:", IMG, "CROP_MODE:", CROP_MODE)
    print("Recommended threshold:", meta.get("recommended_threshold"), "streak:", meta.get("recommended_streak"))

    # Start display + LED
    try:
        start_display()
    except Exception:
        pass
    led_on()

    # Camera start
    if hasattr(line_camera, "start"):
        try:
            line_camera.start()
        except Exception:
            pass

    # Warmup (avoids first call being slow)
    dummy = torch.zeros((1, 3, IMG, IMG), dtype=torch.float32)
    with torch.inference_mode():
        for _ in range(5):
            _ = model(dummy)

    # Timing stats (EMA)
    ema_fps = 0.0
    ema_inf_ms = 0.0
    alpha = 0.10

    frames = 0
    last_print = time.perf_counter()

    try:
        while True:
            t0 = time.perf_counter()
            frame = line_camera.capture_array()
            if frame is None:
                continue

            x = prep(frame)

            t1 = time.perf_counter()
            with torch.inference_mode():
                out = model(x)
                # get silver prob only (faster than full numpy print)
                silver_p = float(torch.softmax(out, dim=1)[0, 1].item())
            t2 = time.perf_counter()

            inf_ms = (t2 - t1) * 1000.0
            loop_dt = max(t2 - t0, 1e-9)
            fps = 1.0 / loop_dt

            ema_fps = fps if frames == 0 else (1 - alpha) * ema_fps + alpha * fps
            ema_inf_ms = inf_ms if frames == 0 else (1 - alpha) * ema_inf_ms + alpha * inf_ms
            frames += 1

            # Display (overlay)
            disp = frame.copy()
            # If display looks “wrong colors”, flip this:
            # disp = cv2.cvtColor(disp, cv2.COLOR_RGB2BGR)

            txt1 = f"silver={silver_p:.3f}  fps={ema_fps:.1f}"
            txt2 = f"infer={ema_inf_ms:.2f}ms  IMG={IMG}  crop={CROP_MODE}"
            cv2.putText(disp, txt1, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 4, cv2.LINE_AA)
            cv2.putText(disp, txt1, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1, cv2.LINE_AA)
            cv2.putText(disp, txt2, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 4, cv2.LINE_AA)
            cv2.putText(disp, txt2, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

            safe_show("silver_test", disp)

            # Print once per ~1s (printing every frame murders speed)
            now = time.perf_counter()
            if now - last_print >= 1.0:
                print(f"silver={silver_p:.3f}  fps={ema_fps:.1f}  infer={ema_inf_ms:.2f}ms")
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        led_off()
        try:
            stop_display()
        except Exception:
            pass
        if hasattr(line_camera, "stop"):
            try:
                line_camera.stop()
            except Exception:
                pass

if __name__ == "__main__":
    main()
