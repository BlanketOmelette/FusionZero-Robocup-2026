#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
import time

# Make imports work when run as: python3 1_robot/tools/xxx.py
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import cv2

from hardware.camera import Camera
from line.victim_detector import VictimModel, VictimDetector


MODEL_PATH_DEFAULT = "/home/fusion/FusionZero-Robocup-2026/imx_model_out/network.rpk"


def main():
    model_path = sys.argv[1] if len(sys.argv) > 1 else MODEL_PATH_DEFAULT

    print("Loading model:", model_path)
    # Works with the VictimModel.load() version
    model = VictimModel.load(model_path)

    print(f"IMX input size: {model.input_w}x{model.input_h}, fps={model.fps}")
    print("Creating evac camera wrapper...")
    evac_cam = Camera("evac")

    print("Starting detector...")
    det = VictimDetector(
        evac_cam,
        model,
        window_name="EVAC_TEST",
        period_s=0.0,      # run inference every frame for this test
    )

    t0 = time.time()
    frames = 0
    try:
        while True:
            det.step()
            frames += 1

            # lightweight progress print once per second
            if time.time() - t0 >= 1.0:
                print("frames:", frames)
                t0 = time.time()
                frames = 0

            # if your VictimDetector.step already does waitKey, this still works
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord("q")):
                break

    finally:
        print("Closing...")
        det.close()


if __name__ == "__main__":
    main()
