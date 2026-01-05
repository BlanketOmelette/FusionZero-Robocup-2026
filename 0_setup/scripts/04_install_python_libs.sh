#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source venv/bin/activate

python -m pip install --upgrade \
  gpiozero lgpio \
  adafruit-blinka \
  adafruit-circuitpython-busdevice \
  adafruit-circuitpython-vl53l0x \
  adafruit-circuitpython-bno08x \
  adafruit-circuitpython-ssd1306 \
  pillow numpy

echo "Python libs installed."
