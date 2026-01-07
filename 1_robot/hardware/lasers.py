from core.shared_imports import time, board, DigitalOutputDevice, adafruit_vl53l0x
from core.utilities import debug


class Lasers:
    """
    3x VL53L0X on one I2C bus:
      forward: no XSHUT, default addr 0x29
      left:    XSHUT, reassigned to 0x30
      right:   XSHUT, reassigned to 0x31
    """

    FWD_ADDR = 0x29
    LEFT_ADDR = 0x30
    RIGHT_ADDR = 0x31
    FWD_TMP_ADDR = 0x32

    def __init__(self) -> None:
        self.i2c = board.I2C()

        # XSHUT is active low: low shuts sensor down
        self._xshut_left = DigitalOutputDevice(10)
        self._xshut_right = DigitalOutputDevice(6)

        self.forward = None
        self.left = None
        self.right = None

        self._init_sensors()

        debug(["INITIALISATION", "TOF", "✓"], [25, 25, 50])

    def _init_sensors(self) -> None:
        # Ensure left/right are off so forward is the only one responding at 0x29
        self._xshut_left.off()
        self._xshut_right.off()
        time.sleep(0.05)

        # Forward at 0x29
        self.forward = adafruit_vl53l0x.VL53L0X(self.i2c, address=self.FWD_ADDR)

        # Move forward away so we can bring up left/right on 0x29 safely
        self.forward.set_address(self.FWD_TMP_ADDR)
        time.sleep(0.01)

        # Left
        self._xshut_left.on()
        time.sleep(0.05)
        self.left = adafruit_vl53l0x.VL53L0X(self.i2c, address=self.FWD_ADDR)
        self.left.set_address(self.LEFT_ADDR)
        time.sleep(0.01)

        # Right
        self._xshut_right.on()
        time.sleep(0.05)
        self.right = adafruit_vl53l0x.VL53L0X(self.i2c, address=self.FWD_ADDR)
        self.right.set_address(self.RIGHT_ADDR)
        time.sleep(0.01)

        # Put forward back to default 0x29 (optional, but nice)
        self.forward.set_address(self.FWD_ADDR)
        time.sleep(0.01)

    def read(self) -> list[int | None]:
        # Returns mm, or None if a sensor errors
        def safe_range(dev):
            try:
                return int(dev.range)
            except Exception:
                return None

        return [safe_range(self.forward), safe_range(self.left), safe_range(self.right)]

    def read_dict(self) -> dict[str, int | None]:
        f, l, r = self.read()
        return {"forward": f, "left": l, "right": r}

    def close(self) -> None:
        # adafruit_vl53l0x objects do not have a real close; just release XSHUT devices
        self._xshut_left.off()
        self._xshut_right.off()
        self._xshut_left.close()
        self._xshut_right.close()
