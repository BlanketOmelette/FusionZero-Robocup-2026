#!/usr/bin/env bash
set -e

echo "[04] Installing Python libraries (system Python, no venv)"

PIP_FLAGS="--upgrade --break-system-packages"

python3 -m pip install $PIP_FLAGS pip setuptools wheel

python3 -m pip install $PIP_FLAGS \
  lgpio \
  adafruit-blinka \
  adafruit-circuitpython-busdevice \
  adafruit-circuitpython-vl53l0x \
  adafruit-circuitpython-bno08x \
  adafruit-circuitpython-ssd1306

echo "[04] Done."
