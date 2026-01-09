from __future__ import annotations

import random
from core.shared_imports import time
from core.listener import listener
from core.utilities import show
from hardware.robot import motors, led, touch, lasers, servos

from line.victim_detector import VictimDetector, VictimModel


TOUCH_PRESSED_VALUE = 1


def _pressed(v: int) -> bool:
    return v == TOUCH_PRESSED_VALUE


def _clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return lo if x < lo else hi if x > hi else x


class Evac:
    def __init__(self) -> None:
        # Entry
        self.entry_speed = 0.50
        self.entry_seconds = 1.0

        # Search states
        self.spin_fast_speed = 0.60
        self.spin_fast_min_s = 6.50
        self.spin_fast_max_s = 6.85

        self.spin_slow_speed = 0.50
        self.spin_slow_min_s = 8.30
        self.spin_slow_max_s = 8.80

        self.move_speed = 0.75
        self.move_min_s = 1.00
        self.move_max_s = 2.00

        self.bump_speed = 0.70
        self.bump_back_s = 0.60
        self.bump_fwd_s = 0.60

        self.state = "SPIN_FAST"
        self.deadline = 0.0
        self.spin_dir = 1

        # Laser based approach tuning (mm)
        self.stop_mm = 50
        self.slow_start_mm = 200
        self.min_fwd = 0.2
        self.max_fwd = 0.60

        self.align_stop_px = 5
        self.initial_align_px = 200

        self._did_entry = False
        self._entry_deadline = 0.0

        self.detector: VictimDetector | None = None
        self.last_dets = []

        # Victim tracking
        self.track_hold_s = 0.25
        self.track_until = 0.0
        self.target_det = None

        self.kp = 0.5

        # display dims
        self.disp_w = 800
        self.disp_h = 600

        self.did_grab = False

    def reset(self) -> None:
        self._did_entry = False
        self._entry_deadline = time.perf_counter() + self.entry_seconds

        self.state = "SPIN_FAST"
        self.deadline = 0.0
        self.spin_dir = 1

        self.last_dets = []
        self.track_until = 0.0
        self.target_det = None
        self.did_grab = False

        motors.run(0.0, 0.0)

    def _set_state(self, name: str, duration_s: float) -> None:
        self.state = name
        self.deadline = time.perf_counter() + duration_s

    def _time_up(self) -> bool:
        return time.perf_counter() >= self.deadline

    def _start_spin_fast(self) -> None:
        self._set_state("SPIN_FAST", random.uniform(self.spin_fast_min_s, self.spin_fast_max_s))

    def _start_spin_slow(self) -> None:
        self._set_state("SPIN_SLOW", random.uniform(self.spin_slow_min_s, self.spin_slow_max_s))

    def _start_move(self) -> None:
        self._set_state("MOVE_FWD", random.uniform(self.move_min_s, self.move_max_s))

    def _touch_interrupt(self) -> None:
        fl, fr, bl, br = touch.read()

        if _pressed(fl) or _pressed(fr):
            self._set_state("BUMP_BACK", self.bump_back_s)
            return

        if _pressed(bl) or _pressed(br):
            self._set_state("BUMP_FWD", self.bump_fwd_s)
            return

    def _front_mm(self) -> int | None:
        f, l, r = lasers.read()
        if f is not None:
            return f
        vals = [v for v in (l, r) if v is not None]
        return min(vals) if vals else None

    def _pick_target(self, dets):
        live = [d for d in dets if d.cls == 1]
        dead = [d for d in dets if d.cls == 0]

        if live:
            cx_screen = 0.5 * self.disp_w

            def live_key(d):
                cx = d.x + 0.5 * d.w
                center_err = abs(cx - cx_screen)          # smaller is better
                return (center_err, -d.score, -(d.w * d.h), -d.h)

            return min(live, key=live_key)

        if dead: return max(dead, key=lambda d: (d.h, d.w * d.h, d.score))

        return None

    def _grab_victim(self) -> None:
        motors.run(0.0, 0.0)
        servos.grab_down()
        motors.run(0.7, 0.7, 0.15)
        motors.run(0.0, 0.0, 0.3)
        servos.grab_dump(1.5)
        servos.grab_partial(0.5)
        servos.grab_dump(0.5)
        print("grabbed")

    def step_motion(self) -> None:
        now = time.perf_counter()

        if self.target_det is not None and now < self.track_until:
            self._touch_interrupt()
            if self.state in ("BUMP_BACK", "BUMP_FWD"):
                return

            d = self.target_det

            cx = d.x + 0.5 * d.w
            x_err_px = cx - (0.5 * self.disp_w)
            abs_err_px = abs(x_err_px)

            err = x_err_px / (0.5 * self.disp_w)
            turn = _clamp(self.kp * err)
            turn = _clamp(turn, -0.45, 0.45)

            dist = self._front_mm()
            if dist is None:
                dist = 9999
            print("Evac distance mm:", dist)

            if dist <= self.stop_mm:
                if abs_err_px <= self.align_stop_px:
                    if not self.did_grab:
                        self.did_grab = True
                        self._grab_victim()
                else: motors.run(-0.25 + turn, -0.25 - turn)
                return

            if abs_err_px > self.initial_align_px: fwd = 0.1
            elif dist >= self.slow_start_mm: fwd = self.max_fwd
            else:
                t = (dist - self.stop_mm) / float(self.slow_start_mm - self.stop_mm)
                t = _clamp(t, 0.0, 1.0)
                fwd = self.min_fwd + t * (self.max_fwd - self.min_fwd)

            left = _clamp(fwd + turn)
            right = _clamp(fwd - turn)
            motors.run(left, right)
            return

        if self.state not in ("BUMP_BACK", "BUMP_FWD"):
            self._touch_interrupt()
            if self.state in ("BUMP_BACK", "BUMP_FWD"):
                return

        if self.state == "SPIN_FAST":
            s = self.spin_fast_speed * self.spin_dir
            motors.run(s, -s)
            if self._time_up():
                self._start_spin_slow()
            return

        if self.state == "SPIN_SLOW":
            s = self.spin_slow_speed * self.spin_dir
            motors.run(s, -s)
            if self._time_up():
                self._start_move()
            return

        if self.state == "MOVE_FWD":
            motors.run(self.move_speed, self.move_speed)
            if self._time_up():
                self._start_spin_fast()
            return

        if self.state == "BUMP_BACK":
            motors.run(-self.bump_speed, -self.bump_speed)
            if self._time_up():
                self._start_spin_fast()
            return

        if self.state == "BUMP_FWD":
            motors.run(self.bump_speed, self.bump_speed)
            if self._time_up():
                self._start_spin_fast()
            return

        motors.run(0.0, 0.0)

    def step_vision(self, victim_model: VictimModel | None) -> None:
        if victim_model is None:
            return

        if self.detector is None:
            self.detector = VictimDetector(
                victim_model,
                crop_top_px=70,
                score_thresh=0.30,
                max_dets=50,
                display_size=(800, 600),
            )

        frame, meta = victim_model.capture()
        if frame is None or meta is None:
            return

        disp, dets = self.detector.process(frame, meta)
        self.last_dets = dets

        show(disp, name="EVAC", display=True)

        H0, W0 = frame.shape[:2]
        self.disp_w = W0
        self.disp_h = max(1, H0 - 70)

        now = time.perf_counter()
        t = self._pick_target(dets)

        if t is not None:
            self.target_det = t
            self.track_until = now + self.track_hold_s
            self.did_grab = False
        elif now >= self.track_until:
            self.target_det = None


_evac: Evac | None = None
_was_mode2 = False


def main(robot_state=None, victim_model: VictimModel | None = None) -> None:
    global _evac, _was_mode2

    mode = listener.get_mode()
    if mode != 2:
        if _was_mode2:
            motors.run(0.0, 0.0)
        _was_mode2 = False
        return

    if _evac is None:
        _evac = Evac()

    if not _was_mode2:
        _evac.reset()
        led.on()
        _was_mode2 = True

    # ENTRY
    if not _evac._did_entry:
        motors.run(_evac.entry_speed, _evac.entry_speed)

        # warm up the model during entry (first call may be slow)
        _evac.step_vision(victim_model)

        if time.perf_counter() >= _evac._entry_deadline:
            motors.run(0.0, 0.0)
            _evac._did_entry = True
            _evac.spin_dir = 1 if lasers.read()[1] < lasers.read()[2] else -1
            _evac._start_spin_fast()
        return

    _evac.step_vision(victim_model)
    _evac.step_motion()
