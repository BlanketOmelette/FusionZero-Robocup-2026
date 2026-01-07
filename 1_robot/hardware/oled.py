from __future__ import annotations

import time
import threading
from pathlib import Path

from core.shared_imports import board, adafruit_ssd1306, Image, ImageDraw, ImageFont
from core.utilities import debug


class OLED:
    """
    Drop-in OLED driver:
      - Same API as your current OLED class (text/clear/display_logo)
      - Internally dedupes and rate-limits display updates
      - Uses a background thread to push frames so your main loop doesn't stall
      - Software flip (ROTATE_180) for reliable upside-down fix
    """

    def __init__(self, addr: int = 0x3C, flip: bool = True, max_hz: float = 3.0):
        self.WIDTH = 128
        self.HEIGHT = 64
        self.flip = bool(flip)

        self.max_hz = float(max_hz)
        self._min_dt = 1.0 / self.max_hz if self.max_hz > 0 else 0.0

        i2c = board.I2C()
        self.oled = adafruit_ssd1306.SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c, addr=addr)

        # Font cache
        self._font_cache: dict[tuple[str, int], ImageFont.ImageFont] = {}

        # Prefer repo-local font if present, fall back to system fonts
        self._font_paths = self._font_candidates()

        # Front buffer (what callers draw into)
        self.image = Image.new("1", (self.WIDTH, self.HEIGHT))
        self.draw = ImageDraw.Draw(self.image)

        # Background worker state
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._dirty = True
        self._last_signature: tuple | None = None
        self._t_last_push = 0.0

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

        self.clear()
        debug(["INITIALISATION", "OLED", "✓"], [25, 25, 50])

    # ---------- font helpers ----------

    def _font_candidates(self) -> list[str]:
        """
        Prefer a repo-bundled JetBrainsMono Nerd Font if you put it in:
          1_robot/assets/fonts/JetBrainsMonoNerdFont-Regular.ttf
        """
        paths: list[str] = []

        try:
            base = Path(__file__).resolve().parents[1]  # 1_robot
            repo_font = base / "assets" / "fonts" / "JetBrainsMonoNerdFont-Regular.ttf"
            if repo_font.exists():
                paths.append(str(repo_font))
        except Exception:
            pass

        # System fallbacks
        paths += [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        ]
        return paths

    def _get_font(self, size: int | None) -> ImageFont.ImageFont:
        if not size or size <= 0:
            return ImageFont.load_default()

        # Cache key based on first working font path + size
        for p in self._font_paths:
            key = (p, int(size))
            if key in self._font_cache:
                return self._font_cache[key]
            try:
                f = ImageFont.truetype(p, int(size))
                self._font_cache[key] = f
                return f
            except Exception:
                continue

        return ImageFont.load_default()

    # ---------- core drawing ----------

    def clear(self) -> None:
        with self._lock:
            self.image = Image.new("1", (self.WIDTH, self.HEIGHT))
            self.draw = ImageDraw.Draw(self.image)
            self._dirty = True

    def text(
        self,
        text: str,
        x: int,
        y: int,
        update_display: bool = True,
        clear: bool = False,
        **kwargs,
    ) -> None:
        """
        Compatible with your old calls.
        Extra supported kwargs (optional):
          - size=int
          - safe_ascii=bool (default True)
        """
        size = kwargs.get("size", None)
        safe_ascii = kwargs.get("safe_ascii", True)

        s = str(text)
        if safe_ascii:
            s = (
                s.replace("✓", "OK")
                 .replace("✗", "X")
                 .replace("✔", "OK")
                 .replace("✘", "X")
            )

        with self._lock:
            if clear:
                self.image = Image.new("1", (self.WIDTH, self.HEIGHT))
                self.draw = ImageDraw.Draw(self.image)

            font = self._get_font(size)
            self.draw.text((x, y), s, fill=255, font=font)

            if update_display:
                # mark dirty and let worker push asynchronously
                self._dirty = True

    def display_logo(self):
        self.clear()
        self.text("Ready!", 0, 0, update_display=True, size=14)

    # ---------- internal worker ----------

    def _signature(self, img: Image.Image) -> tuple:
        # Cheap signature to avoid pushing identical frames.
        # We include bytes and flip so flipped/unflipped don't collide.
        b = img.tobytes()
        return (len(b), hash(b), self.flip)

    def _push_image(self, img: Image.Image) -> None:
        if self.flip:
            img = img.transpose(Image.ROTATE_180)
        try:
            self.oled.image(img)
            self.oled.show()
        except Exception:
            pass

    def _worker(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.01)

            with self._lock:
                if not self._dirty:
                    continue
                img = self.image.copy()
                self._dirty = False

            sig = self._signature(img)
            if sig == self._last_signature:
                continue

            # rate limit physical I2C updates
            now = time.perf_counter()
            dt = now - self._t_last_push
            if self._min_dt > 0 and dt < self._min_dt:
                time.sleep(self._min_dt - dt)

            self._t_last_push = time.perf_counter()
            self._push_image(img)
            self._last_signature = sig

    # ---------- shutdown ----------

    def close(self) -> None:
        self._stop.set()
        try:
            self._thread.join(timeout=0.5)
        except Exception:
            pass
        try:
            self.oled.fill(0)
            self.oled.show()
        except Exception:
            pass
        debug(["TERMINATION", f"OLED", "✓"], [25, 25, 50])
