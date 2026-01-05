#!/usr/bin/env bash
set -e
sudo raspi-config nonint do_i2c 0
echo "I2C enabled. Reboot now."
