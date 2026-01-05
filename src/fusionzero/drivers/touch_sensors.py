from dataclasses import dataclass
from typing import Dict, List, Tuple
from gpiozero import Button

@dataclass(frozen=True)
class TouchPins:
    front_left: int = 23
    front_right: int = 22
    back_left: int = 17
    back_right: int = 5

class TouchSensors:
    """
    Active-low touch modules (common): touched pulls line LOW.
    Untouched -> 1, Touched -> 0 (matches your old code).
    """
    def __init__(self, pins: TouchPins = TouchPins(), bounce_time: float = 0.02):
        self.pins = pins
        self.order: List[Tuple[str, int]] = [
            ("front_left", pins.front_left),
            ("front_right", pins.front_right),
            ("back_left", pins.back_left),
            ("back_right", pins.back_right),
        ]
        self._btn = {name: Button(pin, pull_up=True, bounce_time=bounce_time)
                     for name, pin in self.order}

    def read_raw_list(self) -> List[int]:
        return [0 if self._btn[name].is_pressed else 1 for name, _ in self.order]

    def read_pressed(self) -> Dict[str, bool]:
        return {name: bool(self._btn[name].is_pressed) for name, _ in self.order}
