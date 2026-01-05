#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source venv/bin/activate

# Pi 5 safe GPIO stack
python -m pip install --upgrade gpiozero lgpio

# I2C sensor stack (VL53L0X + SSD1306 OLED)
python -m pip install --upgrade \
  adafruit-blinka \
  adafruit-circuitpython-busdevice \
  adafruit-circuitpython-vl53l0x \
  adafruit-circuitpython-ssd1306 \
  pillow \
  numpy

echo "Python libs installed."
