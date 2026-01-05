#!/usr/bin/env bash
set -e

echo "[02] Enabling I2C"
sudo raspi-config nonint do_i2c 0

# Ensure kernel module loads
if ! grep -q '^i2c-dev' /etc/modules; then
  echo 'i2c-dev' | sudo tee -a /etc/modules >/dev/null
fi

echo "[02] I2C enabled. Reboot recommended."
