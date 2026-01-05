import time
import board, busio
from fusionzero.drivers.tof_vl53l0x_multi import MultiVL53L0X

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    tof = MultiVL53L0X(i2c, left_xshut_gpio=10, right_xshut_gpio=6)

    while True:
        print(tof.read_dict())
        time.sleep(0.1)

if __name__ == "__main__":
    main()
