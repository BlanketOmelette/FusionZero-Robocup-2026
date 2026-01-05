import time
import board, busio
from fusionzero.drivers.bno08x_imu import BNO08xIMU

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    imu = BNO08xIMU(i2c, address=0x4B)

    while True:
        r = imu.read()
        print(
            f"yaw={r.yaw_deg} pitch={r.pitch_deg} roll={r.roll_deg} "
            f"accel={r.accel} gyro={r.gyro}"
        )
        time.sleep(0.1)

if __name__ == "__main__":
    main()
