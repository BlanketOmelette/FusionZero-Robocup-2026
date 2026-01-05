import time
from fusionzero.drivers.motor_tb6612fng import make_drive


def ramp(drive, l0: float, r0: float, l1: float, r1: float, secs: float, steps: int = 40):
    steps = max(1, int(steps))
    dt = float(secs) / steps
    for i in range(steps + 1):
        t = i / steps
        l = l0 + (l1 - l0) * t
        r = r0 + (r1 - r0) * t
        drive.set(l, r)
        time.sleep(dt)


def main():
    drive = make_drive()

    print("Motor bringup (TB6612FNG, PWM)")
    print("Put the bot on a stand. Ctrl+C should stop safely.")

    try:
        drive.coast()
        time.sleep(0.3)

        print("Ramp both forward to 0.55")
        ramp(drive, 0.0, 0.0, 0.55, 0.55, secs=2.0)
        time.sleep(0.6)
        drive.quick_stop()
        time.sleep(0.6)

        print("Ramp both reverse to -0.55")
        ramp(drive, 0.0, 0.0, -0.55, -0.55, secs=2.0)
        time.sleep(0.6)
        drive.quick_stop()

        print("Done")
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stopping")
        drive.quick_stop()
    finally:
        drive.close()


if __name__ == "__main__":
    main()
