# 0_setup (Pi 5)

Run these once on a fresh Raspberry Pi OS install.

Order:
1) bash scripts/1_system_update.sh
2) bash scripts/2_enable_i2c.sh   (reboot after)
3) bash scripts/3_install_robot_packages.sh (apt camera stack etc)
4) bash scripts/4_install_python_libraries.sh (sensors and gpio python libs)
5) bash verify/i2c_scan.sh
6) bash verify/camera_list.sh
