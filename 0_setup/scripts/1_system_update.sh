#!/usr/bin/env bash
set -e

echo "[01] System update + base packages"

sudo apt-get update
sudo apt-get -y full-upgrade

# Core tooling + build deps + diagnostics
sudo apt-get install -y \
  git curl wget unzip zip \
  build-essential pkg-config \
  python3 python3-pip python3-dev \
  i2c-tools v4l-utils \
  x11-apps \
  ffmpeg \
  python3-torch

# Camera stack (Bookworm and later uses rpicam-* apps)
# These tools let you do: rpicam-hello --list-cameras
sudo apt-get install -y \
  rpicam-apps

# Python camera libs (system matched to libcamera stack)
sudo apt-get install -y \
  python3-picamera2 \
  python3-opencv \
  python3-numpy \
  python3-pil

# GPIO
sudo apt-get install -y \
  python3-gpiozero \
  liblgpio-dev

echo "[01] Done. Reboot recommended if kernel/firmware changed."
