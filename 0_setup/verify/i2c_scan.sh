#!/usr/bin/env bash
set -e
echo "I2C scan on bus 1 (common addresses: VL53L0X=0x29, BNO08X often 0x4B, mux often 0x70)"
i2cdetect -y 1
