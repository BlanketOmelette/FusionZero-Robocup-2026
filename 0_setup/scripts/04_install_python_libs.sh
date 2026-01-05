#!/usr/bin/env bash
set -e

cd "$(dirname "$0")/.."

if [ ! -d "venv" ]; then
  echo "venv not found. Run: bash 0_setup/scripts/03_python_venv.sh"
  exit 1
fi

source venv/bin/activate

echo "[04] Installing Python libraries (gpio + sensors)"
python -m pip install --upgrade \
  gpiozero lgpio \
  adafruit-blinka \
  adafruit-circuitpython-busdevice \
  adafruit-circuitpython-vl53l0x \
  adafruit-circuitpython-bno08x \
  adafruit-circuitpython-ssd1306 \
  pillow numpy

echo "[04] Done."
