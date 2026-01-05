#!/usr/bin/env bash
set -e
sudo apt-get update
sudo apt-get -y full-upgrade
sudo apt-get install -y \
  git curl wget unzip zip \
  python3 python3-pip python3-venv python3-dev \
  i2c-tools \
  build-essential
echo "Done. Reboot recommended."
