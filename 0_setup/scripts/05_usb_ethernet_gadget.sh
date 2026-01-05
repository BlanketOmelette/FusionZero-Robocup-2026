#!/usr/bin/env bash
set -e

echo "[05] Installing and enabling USB Ethernet gadget (rpi-usb-gadget)"

sudo apt-get update
sudo apt-get install -y rpi-usb-gadget

# Enable gadget mode
sudo rpi-usb-gadget on

echo
echo "[05] Enabled. Reboot required."
echo "After reboot, connect a USB-C data cable from Pi 5 USB-C port to your laptop."
echo "On the Pi, check: ip -4 a show usb0"
