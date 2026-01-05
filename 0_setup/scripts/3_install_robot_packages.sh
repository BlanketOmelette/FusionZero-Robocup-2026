#!/usr/bin/env bash
set -e

echo "[03] Installing robot Python libraries (no venv)"

# Adafruit CircuitPython libs (VL53L0X, BNO08X, SSD1306 OLED)
# PEP 668 on Bookworm blocks global pip installs unless you pass --break-system-packages.
python3 -m pip install --upgrade --break-system-packages \
  adafruit-blinka \
  adafruit-circuitpython-busdevice \
  adafruit-circuitpython-vl53l0x \
  adafruit-circuitpython-bno08x \
  adafruit-circuitpython-ssd1306 \
  pillow

# lgpio backend for gpiozero on Pi 5 (works well for most GPIO tasks)
python3 -m pip install --upgrade --break-system-packages \
  lgpio

echo "[03] Done."
echo "If you get weird permission errors on I2C, reboot once after enabling I2C."
