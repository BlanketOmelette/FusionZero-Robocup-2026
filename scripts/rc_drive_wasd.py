#!/usr/bin/env python3
import time
import curses
from dataclasses import dataclass
from typing import Optional

from fusionzero.drivers.motor_tb6612fng import make_drive
from fusionzero.drivers.led_gpio import LedGPIO
from fusionzero.drivers.camera import Camera, has_display

# ===== constants (edit these only) =====
USE_CAMERAS = True
SHOW_WINDOWS = True  # only works if DISPLAY exists
WIDE_INDEX = 0
AI_INDEX = 1

DRIVE_SPEED = 0.55
TURN_SPEED = 0.45
COAST_TIMEOUT_S = 0.20


@dataclass
class FpsCounter:
    frames: int = 0
    last_t: float = 0.0
    fps: float = 0.0

    def tick(self, now: float):
        self.frames += 1
        dt = now - self.last_t
        if dt >= 1.0:
            self.fps = self.frames / dt
            self.frames = 0
            self.last_t = now


def clamp(x: float) -> float:
    return max(-1.0, min(1.0, float(x)))


def main():
    drive = make_drive()
    led = LedGPIO(pin_bcm=4, active_high=True, initial_on=False)

    cam_wide: Optional[Camera] = None
    cam_ai: Optional[Camera] = None

    gui = bool(USE_CAMERAS and SHOW_WINDOWS and has_display())

    fps_w = FpsCounter(last_t=time.perf_counter())
    fps_a = FpsCounter(last_t=time.perf_counter())

    if USE_CAMERAS:
        cam_wide = Camera(WIDE_INDEX)  # auto 120 fps for imx708_wide
        cam_ai = Camera(AI_INDEX)      # auto 30 fps for imx500

        if gui:
            import cv2
            cv2.namedWindow("Wide", cv2.WINDOW_NORMAL)
            cv2.namedWindow("AI", cv2.WINDOW_NORMAL)

    def loop(stdscr):
        stdscr.nodelay(True)
        stdscr.keypad(True)
        curses.curs_set(0)

        stdscr.addstr(0, 0, "WASD drive | Space LED | X brake | C coast | Q quit")
        stdscr.addstr(1, 0, f"cams={USE_CAMERAS} gui={gui} wide={WIDE_INDEX} ai={AI_INDEX}")
        stdscr.refresh()

        last_cmd_t = time.perf_counter()

        try:
            drive.coast()
            while True:
                now = time.perf_counter()
                ch = stdscr.getch()

                left = right = None

                if ch != -1:
                    last_cmd_t = now

                    if ch in (ord("q"), ord("Q")):
                        break
                    if ch == ord(" "):
                        led.toggle()
                    elif ch in (ord("x"), ord("X")):
                        drive.quick_stop()
                    elif ch in (ord("c"), ord("C")):
                        drive.coast()
                    elif ch in (ord("w"), ord("W")):
                        left = +DRIVE_SPEED
                        right = +DRIVE_SPEED
                    elif ch in (ord("s"), ord("S")):
                        left = -DRIVE_SPEED
                        right = -DRIVE_SPEED
                    elif ch in (ord("a"), ord("A")):
                        left = -TURN_SPEED
                        right = +TURN_SPEED
                    elif ch in (ord("d"), ord("D")):
                        left = +TURN_SPEED
                        right = -TURN_SPEED

                # safety: no key repeats -> coast
                if (now - last_cmd_t) > COAST_TIMEOUT_S:
                    drive.coast()
                elif left is not None:
                    drive.set(clamp(left), clamp(right))

                # cameras
                if USE_CAMERAS and cam_wide and cam_ai:
                    fw = cam_wide.read()
                    fa = cam_ai.read()
                    fps_w.tick(now)
                    fps_a.tick(now)

                    if gui:
                        import cv2
                        cv2.imshow("Wide", fw)
                        cv2.imshow("AI", fa)
                        cv2.waitKey(1)
                    else:
                        stdscr.addstr(3, 0, f"FPS wide={fps_w.fps:5.1f}  ai={fps_a.fps:5.1f}      ")
                        stdscr.refresh()

                time.sleep(0.005)

        finally:
            try:
                drive.coast()
            except Exception:
                pass

    try:
        curses.wrapper(loop)
    finally:
        if cam_wide:
            cam_wide.close()
        if cam_ai:
            cam_ai.close()
        try:
            led.close()
        except Exception:
            pass
        try:
            drive.close()
        except Exception:
            pass
        if gui:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass


if __name__ == "__main__":
    main()
