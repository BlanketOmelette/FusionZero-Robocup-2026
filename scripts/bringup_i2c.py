import time

import board
import busio

# OLED
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw, ImageFont

# VL53L0X
import adafruit_vl53l0x

# BNO08X
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

def i2c_scan(i2c):
    while not i2c.try_lock():
        time.sleep(0.01)
    try:
        addrs = i2c.scan()
    finally:
        i2c.unlock()
    return addrs


def hex_list(addrs):
    return [f"0x{a:02X}" for a in addrs]


def try_oled(i2c, addr_candidates=(0x3C, 0x3D), w=128, h=64):
    for addr in addr_candidates:
        try:
            display = SSD1306_I2C(w, h, i2c, addr=addr)
            display.fill(0)
            display.show()

            img = Image.new("1", (w, h))
            draw = ImageDraw.Draw(img)
            font = ImageFont.load_default()
            draw.text((0, 0), "OLED OK", font=font, fill=255)
            draw.text((0, 12), f"addr 0x{addr:02X}", font=font, fill=255)

            display.image(img)
            display.show()
            return True, addr
        except Exception as e:
            last = e
    return False, last

def try_vl53l0x(i2c):
    try:
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        # Take a few readings
        readings = []
        for _ in range(5):
            readings.append(sensor.range)
            time.sleep(0.05)
        return True, readings
    except Exception as e:
        return False, e

def try_bno08x(i2c, addr_candidates=(0x4A, 0x4B)):
    last = None
    for addr in addr_candidates:
        try:
            imu = BNO08X_I2C(i2c, address=addr)

            # Enable the same reports as the working bringup script
            imu.enable_feature(BNO_REPORT_ACCELEROMETER)
            imu.enable_feature(BNO_REPORT_GYROSCOPE)
            imu.enable_feature(BNO_REPORT_MAGNETOMETER)
            imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)

            time.sleep(0.2)

            accel = imu.acceleration

            # Some versions expose gyro as .gyroscope, others as .gyro
            gyro = None
            for gname in ("gyroscope", "gyro"):
                if hasattr(imu, gname):
                    try:
                        gyro = getattr(imu, gname)
                    except Exception:
                        gyro = None
                    break

            mag = None
            for mname in ("magnetic", "magnetometer"):
                if hasattr(imu, mname):
                    try:
                        mag = getattr(imu, mname)
                    except Exception:
                        mag = None
                    break

            # Quaternion (this is what your working script uses)
            quat = None
            for qname in ("quaternion", "game_quaternion", "geomagnetic_quaternion"):
                if hasattr(imu, qname):
                    try:
                        quat = getattr(imu, qname)
                    except Exception:
                        quat = None
                    break

            return True, (addr, accel, gyro, mag, quat)

        except Exception as e:
            last = e
    return False, last

def main():
    i2c = busio.I2C(board.SCL, board.SDA)

    print("Scanning I2C...")
    addrs = i2c_scan(i2c)
    print("Found:", hex_list(addrs))

    print("\nOLED test...")
    ok, info = try_oled(i2c)
    print("OLED:", "OK" if ok else "FAIL", info)

    print("\nVL53L0X test...")
    ok, info = try_vl53l0x(i2c)
    print("VL53L0X:", "OK" if ok else "FAIL", info)

   print("\nBNO08X test...")
    ok, info = try_bno08x(i2c)
    if ok:
        addr, accel, gyro, mag, quat = info
        print(f"BNO08X: OK addr 0x{addr:02X}")
        print("  accel:", accel)
        print("  gyro :", gyro if gyro is not None else "not available")
        print("  mag  :", mag if mag is not None else "not available")
        print("  quat :", quat if quat is not None else "not available")
    else:
        print("BNO08X: FAIL", info)




if __name__ == "__main__":
    main()
