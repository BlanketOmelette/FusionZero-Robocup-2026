from __future__ import annotations

import os
import cv2
import numpy as np

try:
    import torch
    import torch.nn as nn
except Exception as e:
    torch = None
    nn = None
    _TORCH_IMPORT_ERROR = e


class SilverLineDetector:
    """
    Minimal TorchScript-first detector.

    Expected input:
      - RGB uint8 numpy array (H,W,3)
      - You can pass an ROI crop (recommended) or the full frame

    Output:
      {'prediction': int, 'class_name': str, 'confidence': float}
    """

    def __init__(
        self,
        model_path: str,
        device: str = "cpu",
        input_size: int = 64,
        num_threads: int = 2,
    ):
        if torch is None:
            raise ImportError(f"torch is not available: {_TORCH_IMPORT_ERROR}")

        self.model_path = model_path
        self.input_size = int(input_size)

        self.device = torch.device(device)
        torch.set_num_threads(int(num_threads))

        self.model = self._load_model(model_path)
        self.model.to(self.device)
        self.model.eval()

        # Class names are assumed in this order
        self.class_names = ["No Silver", "Silver"]

        # Normalization constants (ImageNet style)
        self._mean = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
        self._std = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)

        self._warmup()

    def _load_model(self, model_path: str):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Silver model not found: {model_path}")

        # Preferred: TorchScript
        try:
            return torch.jit.load(model_path, map_location=self.device)
        except Exception:
            pass

        # Fallbacks (only useful if you ever use .pt state dicts again)
        # Keep them here so you can still load old checkpoints if needed.
        try:
            # full model
            m = torch.load(model_path, map_location=self.device)
            return m
        except Exception as e:
            raise RuntimeError(f"Failed to load model (TorchScript/full): {model_path} ({e})")

    def _warmup(self):
        x = torch.randn(1, 3, self.input_size, self.input_size, device=self.device)
        with torch.inference_mode():
            _ = self.model(x)

    def _preprocess(self, rgb: np.ndarray) -> "torch.Tensor":
        # Expect RGB uint8; do NOT convert color here.
        img = cv2.resize(rgb, (self.input_size, self.input_size), interpolation=cv2.INTER_AREA)
        img = img.astype(np.float32) / 255.0
        img = (img - self._mean) / self._std

        x = torch.from_numpy(img).to(self.device)
        x = x.permute(2, 0, 1).unsqueeze(0)  # (1,3,H,W)
        return x

    def predict(self, rgb: np.ndarray) -> dict:
        x = self._preprocess(rgb)

        with torch.inference_mode():
            out = self.model(x)
            probs = torch.softmax(out, dim=1)
            pred = int(torch.argmax(out, dim=1).item())
            conf = float(probs[0, pred].item())

        return {
            "prediction": pred,
            "class_name": self.class_names[pred] if pred < len(self.class_names) else str(pred),
            "confidence": conf,
        }
