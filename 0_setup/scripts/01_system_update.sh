#!/usr/bin/env bash
set -e

echo "[01] System update + base packages"

sudo apt-get update
sudo apt-get -y full-upgrade

# Core tooling + build deps + useful diagnostics
sudo apt-get install -y \
  git curl wget unzip zip \
  python3 python3-pip python3-venv python3-dev \
  i2c-tools \
  build-essential \
  swig \
  liblgpio-dev \
  x11-apps

echo "[01] Done. If kernel/firmware changed, reboot is recommended."
