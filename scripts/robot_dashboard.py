import time
from fusionzero.robot import Robot, RobotConfig

def fmt_angle(x):
    if x is None:
        return "---"
    return f"{x:6.1f}"

def main():
    bot = Robot(RobotConfig(use_oled=True, use_cameras=False))
    while True:
        s = bot.read_sensors()
        tof = s["tof"]
        imu = s["imu"]
        touch = s["touch_raw"]  # [FL, FR, BL, BR]

        lines = [
            f"F {tof['forward']:4d}  L {tof['left']:4d}",
            f"R {tof['right']:4d}  T {touch}",
            f"Y {fmt_angle(imu['yaw'])}",
            f"P {fmt_angle(imu['pitch'])}",
            f"R {fmt_angle(imu['roll'])}",
        ]

        if bot.oled:
            bot.oled.show_lines(lines)

        time.sleep(0.1)

if __name__ == "__main__":
    main()
