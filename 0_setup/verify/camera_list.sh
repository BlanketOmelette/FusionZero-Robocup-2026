#!/usr/bin/env bash
set -e

echo "Listing CSI cameras via rpicam:"
rpicam-hello --list-cameras || true

echo
echo "Listing V4L2 devices:"
v4l2-ctl --list-devices || true

echo
echo "Picamera2 camera info from Python:"
python3 - << 'PY'
from picamera2 import Picamera2
print(Picamera2.global_camera_info())
PY
