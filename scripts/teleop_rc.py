import sys
import time
import termios
import tty
import select
from dataclasses import dataclass

from fusionzero.drivers.motor_tb6612fng import DriveTB6612, TB6612Pins

# If you already have a different LED driver, change this import.
from fusionzero.drivers.led_gpio import LedGPIO


def clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class TeleopState:
    throttle: float = 0.0   # forward/back: + is forward intent
    steer: float = 0.0      # left/right: + means turn right
    max_throttle: float = 0.70
    max_steer: float = 0.60
    step: float = 0.05
    decay: float = 0.92     # gentle auto return to 0 when no keys
    deadband: float = 0.02


class RawTerminal:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self, timeout: float = 0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return None
        return sys.stdin.read(1)


def mix_left_right(throttle: float, steer: float):
    """
    Your current convention:
      forward is left=-1, right=+1

    So for forward throttle>0:
      left should be negative
      right should be positive

    Mapping:
      left  = -(throttle + steer)
      right =  (throttle - steer)
    """
    left = -(throttle + steer)
    right = (throttle - steer)
    return clamp(left), clamp(right)


def main():
    # Motors
    drive = DriveTB6612(
        left=TB6612Pins(pwm=12, in1=21, in2=26),
        right=TB6612Pins(pwm=13, in1=20, in2=16),
        pwm_frequency_hz=10_000,     # 20k failed on your setup, 10k is fine
        default_stop="coast",
        deadband=0.03,
        invert_left=False,
        invert_right=False,
    )

    # LED (MOSFET gate)
    led = LedGPIO(pin=4, active_high=True, initial_on=False)

    st = TeleopState()

    print("\nTeleop RC control")
    print("  W/S: throttle up/down")
    print("  A/D: steer left/right")
    print("  Space: toggle LED")
    print("  B: brake stop   C: coast stop   X: quick_stop")
    print("  Q: quit")
    print("\nPut the bot on a stand first.\n")

    last_print = 0.0

    try:
        with RawTerminal() as rt:
            while True:
                key = rt.read_key(timeout=0.02)

                if key is None:
                    # drift back toward 0 (feels less “sticky”)
                    st.throttle *= st.decay
                    st.steer *= st.decay
                    if abs(st.throttle) < st.deadband:
                        st.throttle = 0.0
                    if abs(st.steer) < st.deadband:
                        st.steer = 0.0
                else:
                    k = key.lower()

                    if k == "q":
                        break

                    elif k == "w":
                        st.throttle = clamp(st.throttle + st.step, -st.max_throttle, st.max_throttle)
                    elif k == "s":
                        st.throttle = clamp(st.throttle - st.step, -st.max_throttle, st.max_throttle)

                    elif k == "a":
                        st.steer = clamp(st.steer - st.step, -st.max_steer, st.max_steer)
                    elif k == "d":
                        st.steer = clamp(st.steer + st.step, -st.max_steer, st.max_steer)

                    elif key == " ":
                        led.toggle()

                    elif k == "b":
                        st.throttle = 0.0
                        st.steer = 0.0
                        drive.brake()

                    elif k == "c":
                        st.throttle = 0.0
                        st.steer = 0.0
                        drive.coast()

                    elif k == "x":
                        st.throttle = 0.0
                        st.steer = 0.0
                        drive.quick_stop()

                    elif k == "\x03":  # Ctrl+C if it comes through
                        break

                left, right = mix_left_right(st.throttle, st.steer)
                drive.set(left, right)

                now = time.time()
                if now - last_print > 0.2:
                    last_print = now
                    sys.stdout.write(
                        f"\rthr={st.throttle:+.2f} steer={st.steer:+.2f} | L={left:+.2f} R={right:+.2f} | LED={'ON ' if led.dev.value else 'OFF'}   "
                    )
                    sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping...")
        try:
            drive.quick_stop()
        except Exception:
            pass
        try:
            led.off()
        except Exception:
            pass
        try:
            drive.close()
        except Exception:
            pass
        try:
            led.close()
        except Exception:
            pass
        print("Done.")


if __name__ == "__main__":
    main()
