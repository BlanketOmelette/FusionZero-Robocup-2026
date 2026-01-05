#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
python3 -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip wheel setuptools
echo "Venv ready at 0_setup/venv. Activate with: source 0_setup/venv/bin/activate"
