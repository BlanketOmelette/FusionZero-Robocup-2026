from dataclasses import dataclass
from typing import Dict, List

from gpiozero import Button


@dataclass
class TouchConfig:
    pull_up: bool = True
    bounce_time: float = 0.02


class TouchArray:
    def __init__(self, pins_bcm: List[int], cfg: TouchConfig = TouchConfig()):
        self.pins = pins_bcm
        self.cfg = cfg
        self.buttons = [
            Button(p, pull_up=cfg.pull_up, bounce_time=cfg.bounce_time)
            for p in pins_bcm
        ]

    def read(self) -> Dict[int, bool]:
        # Returns pressed state per pin as True/False
        return {p: bool(btn.is_pressed) for p, btn in zip(self.pins, self.buttons)}
