import time
from gpiozero import DigitalOutputDevice


class LedGPIO:
    """
    Simple LED (via MOSFET) on a single GPIO pin.
    Supports: on, off, toggle, blink (blocking).
    """

    def __init__(self, pin_bcm: int, active_high: bool = True, initial_on: bool = False):
        self.pin_bcm = pin_bcm
        self.dev = DigitalOutputDevice(
            pin_bcm,
            active_high=active_high,
            initial_value=bool(initial_on),
        )

    def on(self) -> None:
        self.dev.on()

    def off(self) -> None:
        self.dev.off()

    def toggle(self) -> None:
        self.dev.toggle()

    def blink(self, delay_s: float = 0.25, times: int = 3) -> None:
        """
        Blocking blink, like your old style.
        """
        delay_s = max(0.0, float(delay_s))
        times = max(0, int(times))
        for _ in range(times):
            self.on()
            time.sleep(delay_s)
            self.off()
            time.sleep(delay_s)

    def close(self) -> None:
        try:
            self.off()
        finally:
            self.dev.close()
