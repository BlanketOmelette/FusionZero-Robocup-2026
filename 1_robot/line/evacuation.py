from __future__ import annotations

import random
from core.shared_imports import time
from core.listener import listener
from core.utilities import show
from hardware.robot import motors, led, touch, lasers

from line.victim_detector import VictimDetector, VictimModel


TOUCH_PRESSED_VALUE = 1


def _pressed(v: int) -> bool:
    return v == TOUCH_PRESSED_VALUE

def _clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return lo if x < lo else hi if x > hi else x



class Evac:
    def __init__(self) -> None:
        self.entry_speed = 0.90
        self.entry_seconds = 1.5

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
        self.stop_mm = 50           # 5 cm
        self.slow_start_mm = 250    # start slowing down at 25 cm (tune)
        self.min_fwd = 0.12
        self.max_fwd = 0.50         # max forward while tracking

        # Alignment (pixels, in cropped raw frame coords)
        self.align_move_px = 30     # allow forward only if within +-30 px
        self.align_stop_px = 5      # final stop only within +-5 px
        # Alignment hysteresis
        self.align_hold_s = 0.30
        self.aligned_until = 0.0


        self.reached_target = False

        self._did_entry = False
        self._entry_deadline = 0.0

        self.detector: VictimDetector | None = None
        self.last_dets = []
        
        # --- Ball tracking (simple P controller) ---
        self.ball_labels_ready = False
        self.ball_cls_ids: set[int] = set()

        # Victim tracking (P controller)
        self.track_hold_s = 0.25
        self.track_until = 0.0
        self.target_det = None   # best victim detection to chase

        self.kp = 1
        self.base_speed = 0.5

        # stop when close (bbox area ratio on display frame)
        self.close_area = 0.3

        # display dims (matches your detector display_size)
        self.disp_w = 800
        self.disp_h = 600



    def reset(self) -> None:
        self._did_entry = False
        self._entry_deadline = time.perf_counter() + self.entry_seconds
        self.state = "SPIN_FAST"
        self.deadline = 0.0
        self.last_dets = []
        self.track_until = 0.0
        self.target_det = None
        self.reached_target = False
        self.aligned_until = 0.0



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
        f, l, r = lasers.read()  # mm, may be None
        if f is not None:
            return f
        vals = [v for v in (l, r) if v is not None]
        return min(vals) if vals else None


    def _pick_target(self, dets):
        live = [d for d in dets if d.cls == 1]
        dead = [d for d in dets if d.cls == 0]
        pool = live if live else dead
        if not pool:
            return None

        # closest proxy: biggest height, then area, then score
        return max(pool, key=lambda d: (d.h, d.w * d.h, d.score))



    def _ensure_ball_ids(self) -> None:
        if self.ball_labels_ready:
            return
        self.ball_labels_ready = True

        if self.detector is None:
            return

        labels = getattr(self.detector.model, "labels", None)
        if not labels:
            return

        for i, lab in enumerate(labels):
            if lab and "ball" in lab.lower():
                self.ball_cls_ids.add(i)

    def _best_ball_det(self, dets, frame_w: int, frame_h: int):
        # If we couldn't find a "ball" label, do nothing (you can hardcode ids later)
        if not self.ball_cls_ids:
            return None

        best = None
        best_score = -1.0
        for d in dets:
            if d.cls not in self.ball_cls_ids:
                continue
            if d.score > best_score:
                best_score = d.score
                best = d
        return best


    def step_motion(self) -> None:
        now = time.perf_counter()

        if self.reached_target:
            motors.run(0.0, 0.0)
            return

        if self.target_det is not None and now < self.track_until:
            # Touch still has priority
            self._touch_interrupt()
            if self.state in ("BUMP_BACK", "BUMP_FWD"):
                return

            d = self.target_det

            # Compute alignment error in pixels
            cx = d.x + 0.5 * d.w
            x_err_px = cx - (0.5 * self.disp_w)
            abs_err_px = abs(x_err_px)

            # Turn (P)
            err = x_err_px / (0.5 * self.disp_w)  # approx -1..1
            turn = _clamp(self.kp * err)
            turn = _clamp(turn, -0.45, 0.45)

            # Distance from front laser
            dist = self._front_mm()
            if dist is None:
                dist = 9999

            # If we're very close, do not drive forward, just center
            if dist <= self.stop_mm:
                if abs_err_px <= self.align_stop_px:
                    self.reached_target = True
                    motors.run(0.0, 0.0)
                    return

                # pivot only to center
                motors.run(turn, -turn)
                return

            # If we're within the move window, "latch" alignment for a short time
            if abs_err_px <= self.align_move_px:
                self.aligned_until = now + self.align_hold_s

            aligned = now < self.aligned_until

            # Not close yet: only move forward if recently aligned (hysteresis)
            if not aligned:
                motors.run(turn, -turn)
                return


            # Speed ramp: slow down as we approach stop_mm
            if dist >= self.slow_start_mm:
                fwd = self.max_fwd
            else:
                t = (dist - self.stop_mm) / float(self.slow_start_mm - self.stop_mm)  # 0..1
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
                crop_top_px=70,          # crop top 70 px
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

        # Decide whether we're tracking a ball
        self._ensure_ball_ids()

        # IMPORTANT: det coords are in the cropped raw frame coords (before display resize)
        H0, W0 = frame.shape[:2]          # raw frame
        self.disp_w = W0
        self.disp_h = max(1, H0 - 70)     # cropped height

        now = time.perf_counter()
        t = self._pick_target(dets)

        if t is not None:
            self.target_det = t
            self.track_until = now + self.track_hold_s
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

    if not _evac._did_entry:
        motors.run(_evac.entry_speed, _evac.entry_speed)
        if time.perf_counter() >= _evac._entry_deadline:
            motors.run(0.0, 0.0)
            _evac._did_entry = True
            _evac._start_spin_fast()
        return

    _evac.step_vision(victim_model)
    _evac.step_motion()
