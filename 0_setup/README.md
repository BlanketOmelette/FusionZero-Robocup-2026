# 0_setup (Pi 5 + Pi 4 compatible)

Run these once on a fresh Raspberry Pi OS install.

Order:
1) bash scripts/01_system_update.sh
2) bash scripts/02_enable_i2c.sh   (reboot after)
3) bash scripts/03_python_venv.sh
4) bash scripts/04_install_python_libs.sh
5) bash verify/i2c_scan.sh
