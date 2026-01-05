#!/usr/bin/env bash
set -e

sudo apt-get update
sudo apt-get -y full-upgrade

# Core tooling + build deps for gpio libs (Pi 5)
sudo apt-get install -y \
  git curl wget unzip zip \
  python3 python3-pip python3-venv python3-dev \
  i2c-tools \
  build-essential \
  swig \
  liblgpio-dev

echo "Done. Reboot recommended."
