from dataclasses import dataclass
from typing import List

from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw, ImageFont


@dataclass
class OledConfig:
    width: int = 128
    height: int = 64
    address: int = 0x3C


class OledDisplay:
    def __init__(self, i2c, cfg: OledConfig = OledConfig()):
        self.cfg = cfg
        self.disp = SSD1306_I2C(cfg.width, cfg.height, i2c, addr=cfg.address)
        self.font = ImageFont.load_default()
        self.clear()

    def clear(self):
        self.disp.fill(0)
        self.disp.show()

    def show_lines(self, lines: List[str]):
        w, h = self.cfg.width, self.cfg.height
        image = Image.new("1", (w, h), 0)
        draw = ImageDraw.Draw(image)

        y = 0
        for line in lines[:8]:
            draw.text((0, y), str(line)[:24], font=self.font, fill=255)
            y += 10

        self.disp.image(image)
        self.disp.show()
