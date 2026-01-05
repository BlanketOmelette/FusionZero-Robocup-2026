#!/usr/bin/env bash
set -e

echo "[04] Installing Python libraries (system Python, no venv)"

# On Raspberry Pi OS (Bookworm and later), pip may refuse global installs due to PEP 668.
# If you prefer not to use a venv, use --break-system-packages.
PIP_FLAGS="--upgrade --break-system-packages"

# Make sure pip itself exists and is current
python3 -m pip install $PIP_FLAGS pip setuptools wheel

# gpio / hardware access
sudo apt-get update
sudo apt-get install -y python3-gpiozero i2c-tools

# gpiozero backend (Pi 5 uses lgpio nicely)
python3 -m pip install $PIP_FLAGS lgpio

# Adafruit CircuitPython stack for I2C sensors and OLED
python3 -m pip install $PIP_FLAGS \
  adafruit-blinka \
  adafruit-circuitpython-busdevice \
  adafruit-circuitpython-vl53l0x \
  adafruit-circuitpython-bno08x \
  adafruit-circuitpython-ssd1306 \
  pillow

# Common numerics you use elsewhere
sudo apt-get install -y python3-numpy

echo "[04] Done."
echo "If you just enabled I2C, reboot once before testing sensors."
