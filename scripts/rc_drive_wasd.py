import time
import math
import curses

from fusionzero.drivers.motor_tb6612fng import make_drive
from fusionzero.drivers.led_gpio import LedGPIO

# ====== CONFIG ======
ENABLE_CAMERAS = True     # constant at top, no runtime toggles
SHOW_WINDOWS = True       # only works if DISPLAY is available
CAM_WIDE_INDEX = 0
CAM_AI_INDEX = 1

MAX_SPEED = 0.75          # cap for safety
TURN_MIX = 0.65           # how strong A/D turns are
RAMP_PER_SEC = 2.5        # 0..1 per second ramp rate (smooth start/stop)
PRINT_FPS_EVERY_S = 1.0
# ====================

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def mix(throttle, steer):
    # throttle forward/back, steer left/right
    left = throttle + steer
    right = throttle - steer
    m = max(1.0, abs(left), abs(right))
    return left / m, right / m


def ramp_toward(cur, target, max_delta):
    if target > cur:
        return min(target, cur + max_delta)
    return max(target, cur - max_delta)


def main(stdscr):
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.curs_set(0)

    drive = make_drive()
    led = LedGPIO(pin_bcm=4, active_high=True, initial_on=False)

    cam_wide = None
    cam_ai = None
    gui = False
    cv2 = None

    if ENABLE_CAMERAS:
        from fusionzero.drivers.camera import Camera, has_display
        cam_wide = Camera(CAM_WIDE_INDEX)
        cam_ai = Camera(CAM_AI_INDEX)

        gui = bool(SHOW_WINDOWS and has_display())
        if gui:
            import cv2 as _cv2
            cv2 = _cv2
            cv2.namedWindow("wide", cv2.WINDOW_NORMAL)
            cv2.namedWindow("ai", cv2.WINDOW_NORMAL)

    target_throttle = 0.0
    target_steer = 0.0
    cur_l = 0.0
    cur_r = 0.0

    last_fps = time.perf_counter()
    last = time.perf_counter()

    stdscr.addstr(0, 0, "RC Drive: WASD move | Space LED toggle | Q quit")
    stdscr.addstr(1, 0, f"Cameras: {'ON' if ENABLE_CAMERAS else 'OFF'} | GUI: {'ON' if gui else 'OFF'}")
    stdscr.refresh()

    try:
        while True:
            now = time.perf_counter()
            dt = now - last
            last = now

            # reset targets each loop, then set if keys are pressed
            target_throttle = 0.0
            target_steer = 0.0

            key = stdscr.getch()
            # read all pending keys (so it feels responsive)
            pressed = set()
            while key != -1:
                pressed.add(key)
                key = stdscr.getch()

            # quit
            if ord("q") in pressed or ord("Q") in pressed:
                break

            # LED toggle
            if ord(" ") in pressed:
                led.toggle()

            # WASD
            if ord("w") in pressed or ord("W") in pressed:
                target_throttle += 1.0
            if ord("s") in pressed or ord("S") in pressed:
                target_throttle -= 1.0
            if ord("a") in pressed or ord("A") in pressed:
                target_steer += 1.0
            if ord("d") in pressed or ord("D") in pressed:
                target_steer -= 1.0

            target_throttle *= MAX_SPEED
            target_steer *= (MAX_SPEED * TURN_MIX)

            target_l, target_r = mix(target_throttle, target_steer)

            # ramp
            max_delta = RAMP_PER_SEC * dt
            cur_l = ramp_toward(cur_l, target_l, max_delta)
            cur_r = ramp_toward(cur_r, target_r, max_delta)

            drive.set(cur_l, cur_r)

            # cameras
            if ENABLE_CAMERAS and cam_wide and cam_ai:
                f0 = cam_wide.read()
                f1 = cam_ai.read()

                if gui and cv2 is not None:
                    # frames are RGB; OpenCV expects BGR for correct colors
                    if f0 is not None:
                        cv2.imshow("wide", cv2.cvtColor(f0, cv2.COLOR_RGB2BGR))
                    if f1 is not None:
                        cv2.imshow("ai", cv2.cvtColor(f1, cv2.COLOR_RGB2BGR))
                    cv2.waitKey(1)

                if now - last_fps >= PRINT_FPS_EVERY_S:
                    stdscr.addstr(3, 0, f"Drive L={cur_l:+.2f} R={cur_r:+.2f}    ")
                    stdscr.addstr(4, 0, f"Cam FPS wide={cam_wide.fps:.1f}  ai={cam_ai.fps:.1f}    ")
                    stdscr.refresh()
                    last_fps = now
            else:
                if now - last_fps >= PRINT_FPS_EVERY_S:
                    stdscr.addstr(3, 0, f"Drive L={cur_l:+.2f} R={cur_r:+.2f}    ")
                    stdscr.refresh()
                    last_fps = now

            time.sleep(0.005)

    finally:
        # safe stop
        try:
            drive.quick_stop()
        except Exception:
            pass
        drive.close()
        led.close()

        if ENABLE_CAMERAS and cam_wide and cam_ai:
            cam_wide.close()
            cam_ai.close()
        if gui and cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    curses.wrapper(main)
