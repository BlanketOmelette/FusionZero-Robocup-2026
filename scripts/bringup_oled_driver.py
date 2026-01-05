import time
import board, busio
from fusionzero.drivers.oled_ssd1306 import OledDisplay, OledConfig

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    oled = OledDisplay(i2c, OledConfig(address=0x3C))

    n = 0
    while True:
        oled.show_lines([
            "OLED OK",
            f"count: {n}",
            "FusionZero 2026",
        ])
        n += 1
        time.sleep(0.5)

if __name__ == "__main__":
    main()
