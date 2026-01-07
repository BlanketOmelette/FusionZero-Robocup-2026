from core.shared_imports import time, HardwarePWM, DigitalOutputDevice
from core.utilities import debug
from core.listener import listener

LEFT_IN1, LEFT_IN2 = 21, 26
RIGHT_IN1, RIGHT_IN2 = 20, 16

LEFT_PWM, RIGHT_PWM = 0, 1

FREQUENCY = 50000
DEADBAND = 0.03

INVERT_LEFT, INVERT_RIGHT = True, False


def _clamp(x: float) -> float:
    if x < -1.0:
        return -1.0
    if x > 1.0:
        return 1.0
    return x


class Side:
    def __init__(self, in1_pin, in2_pin, pwm_channel, frequency, invert: bool) -> None:
        self.in1 = DigitalOutputDevice(in1_pin)
        self.in2 = DigitalOutputDevice(in2_pin)
        self.pwm = HardwarePWM(pwm_channel=pwm_channel, hz=frequency)
        self.pwm.start(0)
        self.invert = invert

    def run(self, speed: float) -> None:
        speed = _clamp(speed)
        if self.invert:
            speed = -speed

        if abs(speed) < DEADBAND:
            self.in1.off()
            self.in2.off()
            self.pwm.change_duty_cycle(0)
            return

        if speed > 0:
            self.in1.on()
            self.in2.off()
        else:
            self.in1.off()
            self.in2.on()

        self.pwm.change_duty_cycle(int(abs(speed) * 100))

    def close(self) -> None:
        try:
            self.pwm.change_duty_cycle(0)
        except Exception:
            pass
        try:
            self.pwm.stop()
        except Exception:
            pass
        self.in1.close()
        self.in2.close()


class Motors:
    def __init__(self) -> None:
        self.left = Side(LEFT_IN1, LEFT_IN2, LEFT_PWM, FREQUENCY, INVERT_LEFT)
        self.right = Side(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, FREQUENCY, INVERT_RIGHT)
        debug(["INITIALISATION", "MOTORS", "✓"], [25, 25, 50])

    def run(self, left_speed: float, right_speed: float, delay: float = 0.0) -> None:
        self.left.run(left_speed)
        self.right.run(right_speed)

        if delay > 0:
            start = time.perf_counter()
            while listener.get_mode() != 0 and not listener.exit_event.is_set():
                if (time.perf_counter() - start) >= delay:
                    break
                time.sleep(0.005)

    def close(self) -> None:
        self.left.close()
        self.right.close()
        debug(["TERMINATION", "MOTORS", "✓"], [25, 25, 50])