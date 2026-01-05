from __future__ import annotations

import time
from typing import Literal, Optional

from gpiozero import DigitalOutputDevice

try:
    from rpi_hardware_pwm import HardwarePWM
except Exception as e:
    HardwarePWM = None
    _IMPORT_ERR = e


StopMode = Literal["coast", "brake"]

# =========================
# DEFAULTS
# =========================
# Direction pins
LEFT_IN1 = 21
LEFT_IN2 = 26
RIGHT_IN1 = 20
RIGHT_IN2 = 16

# Hardware PWM channels (with pwm-2chan overlay)
# GPIO12 -> PWM channel 0
# GPIO13 -> PWM channel 1
LEFT_PWM_CH = 0
RIGHT_PWM_CH = 1

PWM_HZ = 50_000
DEADBAND = 0.03
DEFAULT_STOP: StopMode = "coast"

# Make forward be (+,+)
INVERT_LEFT = False
INVERT_RIGHT = True

PWM_CHIP = 0  # usually 0


def _clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


class _Side:
    """One side of the drive: Hardware PWM + IN1 + IN2. STBY assumed tied HIGH."""

    def __init__(
        self,
        pwm_channel: int,
        in1_pin: int,
        in2_pin: int,
        *,
        pwm_hz: int,
        invert: bool,
        deadband: float,
        default_stop: StopMode,
        chip: int,
    ):
        if HardwarePWM is None:
            raise RuntimeError(
                f"rpi-hardware-pwm not available: {_IMPORT_ERR}. "
                "Install it or use the lgpio PWM driver."
            )

        self.invert = bool(invert)
        self.deadband = float(deadband)
        self.default_stop = default_stop

        self.in1 = DigitalOutputDevice(in1_pin, initial_value=False)
        self.in2 = DigitalOutputDevice(in2_pin, initial_value=False)

        self.pwm = HardwarePWM(pwm_channel=int(pwm_channel), hz=int(pwm_hz), chip=int(chip))
        self.pwm.start(0)  # duty percent

        self.coast()

    def set(self, speed: float, *, stop: Optional[StopMode] = None) -> None:
        stop_mode = stop or self.default_stop
        s = float(speed)
        if self.invert:
            s = -s
        s = _clamp(s)

        if abs(s) < self.deadband:
            self.pwm.change_duty_cycle(0)
            if stop_mode == "brake":
                self.in1.on()
                self.in2.on()
            else:
                self.in1.off()
                self.in2.off()
            return

        # direction
        if s > 0:
            self.in1.on()
            self.in2.off()
        else:
            self.in1.off()
            self.in2.on()

        duty = abs(s) * 100.0
        self.pwm.change_duty_cycle(duty)

    def brake(self) -> None:
        self.pwm.change_duty_cycle(0)
        self.in1.on()
        self.in2.on()

    def coast(self) -> None:
        self.pwm.change_duty_cycle(0)
        self.in1.off()
        self.in2.off()

    def close(self) -> None:
        try:
            self.coast()
        finally:
            try:
                self.pwm.stop()
            except Exception:
                pass
            for d in (self.in1, self.in2):
                try:
                    d.close()
                except Exception:
                    pass


class Drive:
    def __init__(
        self,
        *,
        pwm_hz: int = PWM_HZ,
        deadband: float = DEADBAND,
        default_stop: StopMode = DEFAULT_STOP,
        invert_left: bool = INVERT_LEFT,
        invert_right: bool = INVERT_RIGHT,
        chip: int = PWM_CHIP,
    ):
        self.left = _Side(
            LEFT_PWM_CH, LEFT_IN1, LEFT_IN2,
            pwm_hz=pwm_hz, invert=invert_left, deadband=deadband,
            default_stop=default_stop, chip=chip
        )
        self.right = _Side(
            RIGHT_PWM_CH, RIGHT_IN1, RIGHT_IN2,
            pwm_hz=pwm_hz, invert=invert_right, deadband=deadband,
            default_stop=default_stop, chip=chip
        )

    def set(self, left_speed: float, right_speed: float, *, stop: Optional[StopMode] = None) -> None:
        self.left.set(left_speed, stop=stop)
        self.right.set(right_speed, stop=stop)

    def brake(self) -> None:
        self.left.brake()
        self.right.brake()

    def coast(self) -> None:
        self.left.coast()
        self.right.coast()

    def quick_stop(self, brake_s: float = 0.12) -> None:
        self.brake()
        time.sleep(max(0.0, float(brake_s)))
        self.coast()

    def close(self) -> None:
        self.left.close()
        self.right.close()


def make_drive() -> Drive:
    return Drive()
