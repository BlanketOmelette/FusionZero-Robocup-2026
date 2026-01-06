from core.shared_imports import board, adafruit_ssd1306, Image, ImageDraw, ImageFont
from core.utilities import debug


class OLED:
    def __init__(self):
        self.WIDTH = 128
        self.HEIGHT = 64

        i2c = board.I2C()
        self.oled = adafruit_ssd1306.SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c, addr=0x3C)

        self.font = ImageFont.load_default()
        self.clear()

        debug(["INITIALISATION", "OLED", "✓"], [25, 25, 50])

    def clear(self) -> None:
        self.image = Image.new("1", (self.WIDTH, self.HEIGHT))
        self.draw = ImageDraw.Draw(self.image)
        self.oled.fill(0)
        self.oled.show()

    def text(self, text: str, x: int, y: int, update_display: bool = True, clear: bool = False) -> None:
        if clear:
            self.clear()
        self.draw.text((x, y), str(text), fill=255, font=self.font)
        if update_display:
            self.oled.image(self.image)
            self.oled.show()

    def display_logo(self):
        self.clear()
        self.text("Ready!", 0, 0)
