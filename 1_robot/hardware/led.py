from core.shared_imports import DigitalOutputDevice, time
from core.utilities import debug


class LED:
    def __init__(self) -> None:
        self.pin_bcm = 4
        self.led = DigitalOutputDevice(self.pin_bcm, active_high=True, initial_value=bool(False))
        debug(["INITIALISATION", "LED", "✓"], [25, 25, 50])

    def on(self) -> None:
        self.led.on()

    def off(self) -> None:
        self.led.off()

    def toggle(self) -> None:
        self.led.toggle()

    def blink(self, delay: float = 0.25, times: int = 3) -> None:
        delay = max(0.0, float(delay))
        times = max(0, int(times))
        for _ in range(times):
            self.on()
            time.sleep(delay)
            self.off()
            time.sleep(delay)

    def close(self) -> None:
        try:
            self.off()
        finally:
            self.led.close()
