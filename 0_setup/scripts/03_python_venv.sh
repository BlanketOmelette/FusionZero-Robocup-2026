#!/usr/bin/env bash
set -e

cd "$(dirname "$0")/.."

echo "[03] Creating/using venv at 0_setup/venv"
python3 -m venv venv

source venv/bin/activate
python -m pip install --upgrade pip wheel setuptools

echo "[03] Done. Activate with: source 0_setup/venv/bin/activate"
