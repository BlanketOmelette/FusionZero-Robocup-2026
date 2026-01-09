from __future__ import annotations

from core.shared_imports import time, Servo, LGPIOFactory
from core.utilities import debug

GRAB_PIN = 7
DUMP_PIN = 8

DS3218_MIN_PW = 0.0005
DS3218_MAX_PW = 0.0025

SG90_MIN_PW = 0.0005
SG90_MAX_PW = 0.0024


def _clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


class SoftServo:
    def __init__(
        self,
        pin: int,
        *,
        min_pulse_width: float,
        max_pulse_width: float,
        max_angle_deg: float,
        factory: LGPIOFactory,
        invert: bool = False,
    ) -> None:
        self.max_angle_deg = max_angle_deg
        self.invert = invert
        self._servo = Servo(
            pin,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
            pin_factory=factory,
        )
        self._servo.detach()

    def detach(self) -> None:
        self._servo.detach()

    def set_angle(self, deg: float) -> None:
        deg = _clamp(deg, 0.0, self.max_angle_deg)
        v = (deg / self.max_angle_deg) * 2.0 - 1.0
        if self.invert:
            v = -v
        self._servo.value = _clamp(v, -1.0, 1.0)

    # angle -> sleep -> detach
    def move(self, deg: float, sleep_s: float) -> None:
        self.set_angle(deg)
        if sleep_s > 0:
            time.sleep(sleep_s)
        self.detach()

    def close(self) -> None:
        try:
            self._servo.detach()
        except Exception:
            pass
        try:
            self._servo.close()
        except Exception:
            pass


class Servos:
    def __init__(self) -> None:
        factory = LGPIOFactory()

        self.grab = SoftServo(
            GRAB_PIN,
            min_pulse_width=DS3218_MIN_PW,
            max_pulse_width=DS3218_MAX_PW,
            max_angle_deg=270.0,
            factory=factory,
        )

        self.dump = SoftServo(
            DUMP_PIN,
            min_pulse_width=SG90_MIN_PW,
            max_pulse_width=SG90_MAX_PW,
            max_angle_deg=180.0,
            factory=factory,
        )

        # ----- your final values -----
        self.GRAB_DOWN = 230.0
        self.GRAB_UP = 160.0
        self.GRAB_PARTIAL = 100
        self.GRAB_DUMP = 70.0

        self.DUMP_RIGHT = 47.0
        self.DUMP_CENTER = 83.0
        self.DUMP_LEFT = 110.0

        self.SLEEP_GRAB = 1.5
        self.SLEEP_DUMP = 0.3

        debug(["INITIALISATION", "SERVOS", "✓"], [25, 25, 50])

    # Grab helpers
    def grab_down(self) -> None:
        self.grab.move(self.GRAB_DOWN, sleep_s=self.SLEEP_GRAB)

    def grab_up(self) -> None:
        self.grab.move(self.GRAB_UP, sleep_s=self.SLEEP_GRAB)
    
    def grab_partial(self, delay) -> None:
        self.grab.move(self.GRAB_PARTIAL, sleep_s=delay)

    def grab_dump(self, delay) -> None:
        self.grab.move(self.GRAB_DUMP, sleep_s=delay)

    # Dump helpers
    def dump_right(self) -> None:
        self.dump.move(self.DUMP_RIGHT, sleep_s=self.SLEEP_DUMP)

    def dump_center(self) -> None:
        self.dump.move(self.DUMP_CENTER, sleep_s=self.SLEEP_DUMP)

    def dump_left(self) -> None:
        self.dump.move(self.DUMP_LEFT, sleep_s=self.SLEEP_DUMP)

    def close(self) -> None:
        self.grab.close()
        self.dump.close()
        debug(["TERMINATION", "SERVOS", "✓"], [25, 25, 50])
