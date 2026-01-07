import time
import os
import cv2
import numpy as np
import torch

from hardware.robot import line_camera

MODEL = "1_robot/models/silver_detector.ts"
IMG = 64

mean = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
std  = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)

def prep(rgb):
    h = rgb.shape[0]
    roi = rgb[h // 2 :, :]  # match what line.py does
    img = cv2.resize(roi, (IMG, IMG), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0
    img = (img - mean) / std
    x = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0)
    return x

def main():
    if not os.path.exists(MODEL):
        raise FileNotFoundError(MODEL)

    model = torch.jit.load(MODEL, map_location="cpu")
    model.eval()

    if hasattr(line_camera, "start"):
        try:
            line_camera.start()
        except Exception:
            pass

    while True:
        frame = line_camera.capture_array()
        if frame is None:
            time.sleep(0.01)
            continue

        x = prep(frame)
        with torch.inference_mode():
            out = model(x)
            prob = torch.softmax(out, dim=1)[0].numpy()

        print(f"no_silver={float(prob[0]):.3f}  silver={float(prob[1]):.3f}")
        time.sleep(0.1)

if __name__ == "__main__":
    main()
