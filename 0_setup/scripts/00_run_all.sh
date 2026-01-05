#!/usr/bin/env bash
set -e
bash "$(dirname "$0")/01_system_update.sh"
bash "$(dirname "$0")/02_enable_i2c.sh"
echo
echo "Reboot now, then run:"
echo "  bash 0_setup/scripts/03_python_venv.sh"
echo "  bash 0_setup/scripts/04_install_python_libs.sh"
