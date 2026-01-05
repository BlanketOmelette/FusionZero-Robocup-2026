import time
from dataclasses import dataclass
from typing import Dict, Optional

import adafruit_vl53l0x
from gpiozero import DigitalOutputDevice


@dataclass
class ToFReadings:
    forward: int
    left: int
    right: int


class MultiVL53L0X:
    """
    3x VL53L0X setup:
      - forward has no XSHUT (always on, should reset to 0x29 after power cycle)
      - left XSHUT (BCM pin), final addr 0x30
      - right XSHUT (BCM pin), final addr 0x31

    This driver:
      - scans for forward at 0x29 or TEMP_FORWARD (0x32) to recover after a crash
      - keeps left/right OFF during initial forward detection
    """

    def __init__(
        self,
        i2c,
        left_xshut_gpio: int,
        right_xshut_gpio: int,
        addr_forward: int = 0x29,
        addr_left: int = 0x30,
        addr_right: int = 0x31,
        temp_forward: int = 0x32,
        settle_s: float = 0.5,
        retries: int = 25,
    ):
        self.i2c = i2c
        self.addr_forward = addr_forward
        self.addr_left = addr_left
        self.addr_right = addr_right
        self.temp_forward = temp_forward

        self.left_xshut = DigitalOutputDevice(left_xshut_gpio, active_high=True, initial_value=False)
        self.right_xshut = DigitalOutputDevice(right_xshut_gpio, active_high=True, initial_value=False)

        # Keep left/right OFF so they can't sit on 0x29 and confuse init
        self.left_xshut.off()
        self.right_xshut.off()
        time.sleep(0.2)

        # Forward might be at 0x29 (fresh boot) or stuck at 0x32 (previous run)
        self.forward = self._init_forward(retries=retries)

        # Park forward to temp so 0x29 becomes free
        if self._current_addr_is(self.addr_forward):
            self.forward.set_address(self.temp_forward)
            time.sleep(0.2)

        # Bring up left -> 0x30
        self.left_xshut.on()
        time.sleep(settle_s)
        self.left = self._init_at_0x29(retries=retries)
        self.left.set_address(self.addr_left)
        time.sleep(0.2)

        # Bring up right -> 0x31
        self.right_xshut.on()
        time.sleep(settle_s)
        self.right = self._init_at_0x29(retries=retries)
        self.right.set_address(self.addr_right)
        time.sleep(0.2)

        # Put forward back to 0x29 (if currently at temp)
        # We can’t directly query the addr from the object, so we just try to set.
        try:
            self.forward.set_address(self.addr_forward)
            time.sleep(0.2)
        except Exception:
            # If this fails, forward was probably already at 0x29
            pass

    def _scan(self):
        while not self.i2c.try_lock():
            time.sleep(0.01)
        try:
            return sorted(self.i2c.scan())
        finally:
            self.i2c.unlock()

    def _current_addr_is(self, addr: int) -> bool:
        return addr in self._scan()

    def _init_at_0x29(self, retries: int = 25):
        last = None
        for _ in range(retries):
            try:
                return adafruit_vl53l0x.VL53L0X(self.i2c)  # defaults to 0x29
            except Exception as e:
                last = e
                time.sleep(0.1)
        raise last

    def _init_forward(self, retries: int = 25):
        last = None
        for _ in range(retries):
            addrs = self._scan()
            if self.addr_forward in addrs:
                try:
                    return adafruit_vl53l0x.VL53L0X(self.i2c)
                except Exception as e:
                    last = e
            elif self.temp_forward in addrs:
                # forward is stuck at temp_forward, create object by temporarily telling lib the address
                try:
                    s = adafruit_vl53l0x.VL53L0X(self.i2c)
                    # If this line runs, it means 0x29 exists, but it shouldn't. keep going.
                except Exception:
                    pass

                # Manual attach at 0x32: create at 0x29 is not possible, so force a reset recommendation
                last = ValueError("Forward sensor appears to be at 0x32. Power cycle ToF sensors to reset to 0x29.")
            else:
                last = ValueError("No forward ToF found at 0x29 (or 0x32). Check power/SDA/SCL.")
            time.sleep(0.1)
        raise last

    def read_dict(self) -> Dict[str, int]:
        return {
            "forward": int(self.forward.range),
            "left": int(self.left.range),
            "right": int(self.right.range),
        }
