# 1_robot/line/evacuation.py

from __future__ import annotations

from enum import Enum, auto

from core.shared_imports import time, np
from core.utilities import show, debug_lines, health_tick
from core.listener import listener
from hardware.robot import motors, lasers, touch, imu, line_camera, led


def _clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _mm(v: int | None, default: int = 9999) -> int:
    return default if v is None else int(v)


def _pressed(v: int) -> bool:
    # touch.read(): released=1, pressed=0
    return v == 0


class EvacState(Enum):
    ENTER = auto()
    FOLLOW = auto()
    TURN = auto()
    HIT_RECOVER = auto()
    TILT_RECOVER = auto()
    FIND_EXIT = auto()
    DONE = auto()


class EvacController:
    # -------- tuning knobs (start here on field) --------
    BASE_FWD = 0.35
    TURN_SPEED = 0.32

    # wall follow (right wall)
    WALL_SETPOINT_MM = 140
    KP_WALL = 0.006
    TURN_MAX = 0.28

    # obstacle thresholds
    FRONT_STOP_MM = 160
    RIGHT_LOST_MM = 380

    # timings
    ENTER_SECONDS = 0.70
    TURN_SECONDS = 0.55
    HIT_BACK_SECONDS = 0.35
    HIT_TURN_SECONDS = 0.45
    TILT_BACK_SECONDS = 0.45
    TILT_TURN_SECONDS = 0.50

    # tilt
    TILT_LIMIT_DEG = 18

    # exit detection (simple, tune later)
    EXIT_BLACK_RATIO = 0.22
    EXIT_STREAK = 4

    def __init__(self) -> None:
        self.state = EvacState.ENTER
        self.t0 = time.perf_counter()
        self.turn_dir = -1  # -1 left, +1 right
        self.exit_seen = 0

    def reset(self) -> None:
        self.state = EvacState.ENTER
        self.t0 = time.perf_counter()
        self.turn_dir = -1
        self.exit_seen = 0

    def _set(self, s: EvacState) -> None:
        if s != self.state:
            self.state = s
            self.t0 = time.perf_counter()

    def _t(self) -> float:
        return time.perf_counter() - self.t0

    # ---------- sensors ----------
    def read_tof(self) -> dict[str, int | None]:
        return lasers.read_dict()  # {"forward": mm|None, "left": mm|None, "right": mm|None}

    def read_touch(self) -> list[int]:
        return touch.read()  # [FL, FR, BL, BR]

    def read_imu(self) -> list[int]:
        ang = imu.read()
        return [0, 0, 0] if ang is None else ang

    # ---------- helpers ----------
    def any_hit(self, t: list[int]) -> bool:
        return any(_pressed(x) for x in t)

    def front_hit(self, t: list[int]) -> bool:
        return _pressed(t[0]) or _pressed(t[1])

    def back_hit(self, t: list[int]) -> bool:
        return _pressed(t[2]) or _pressed(t[3])

    def preferred_turn_from_hit(self, t: list[int]) -> int:
        fl, fr, bl, br = t
        # If FL hits, turn right. If FR hits, turn left. Same idea for rear.
        if _pressed(fl) and not _pressed(fr):
            return +1
        if _pressed(fr) and not _pressed(fl):
            return -1
        if _pressed(bl) and not _pressed(br):
            return +1
        if _pressed(br) and not _pressed(bl):
            return -1
        return self.turn_dir

    def tilt_bad(self, ang: list[int]) -> bool:
        roll, pitch, _yaw = ang
        return abs(roll) >= self.TILT_LIMIT_DEG or abs(pitch) >= self.TILT_LIMIT_DEG

    def detect_exit_line(self, line_frame) -> bool:
        """
        Cheap placeholder: look for a strong black band in the bottom of the line camera.
        Assumes RGB uint8, uses channel 0 as luma-ish proxy.
        """
        if line_frame is None:
            return False
        h = line_frame.shape[0]
        band = line_frame[int(h * 0.70) :, :]
        if band.size == 0:
            return False
        ch = band[..., 0]
        black = ch < 60
        ratio = float(black.mean())
        return ratio > self.EXIT_BLACK_RATIO

    def make_debug_frame(self, evac_frame, line_frame):
        # Your current show() keeps only the last queued frame, so combine into one image.
        if evac_frame is None:
            return line_frame
        if line_frame is None:
            return evac_frame
        try:
            return np.concatenate((evac_frame, line_frame), axis=1)
        except Exception:
            return evac_frame

    # ---------- main step ----------
    def step(self, robot_state, evac_frame, line_frame) -> None:
        # hard safety: if not in evac mode, stop and do nothing
        if listener.get_mode() != 2:
            motors.run(0, 0)
            return

        tof = self.read_tof()
        t = self.read_touch()
        ang = self.read_imu()

        fwd = _mm(tof.get("forward"))
        left = _mm(tof.get("left"))
        right = _mm(tof.get("right"))

        # debug frame (combined)
        dbg = self.make_debug_frame(evac_frame, line_frame)
        if dbg is not None:
            show(dbg, name="evac", display=True, debug_lines=None)

        # robot_state book-keeping
        if hasattr(robot_state, "tick"):
            robot_state.tick()

        if hasattr(robot_state, "debug_text"):
            robot_state.debug_text.append(f"EVAC:{self.state.name}")
            robot_state.debug_text.append(f"TOF F:{fwd} L:{left} R:{right}")
            robot_state.debug_text.append(f"TOUCH {t}")
            robot_state.debug_text.append(f"IMU {ang}")

        # safety overrides
        if self.tilt_bad(ang):
            self._set(EvacState.TILT_RECOVER)

        if self.any_hit(t):
            self.turn_dir = self.preferred_turn_from_hit(t)
            self._set(EvacState.HIT_RECOVER)

        # exit detection (only when moving normally)
        if self.state in (EvacState.FOLLOW, EvacState.FIND_EXIT):
            if self.detect_exit_line(line_frame):
                self.exit_seen += 1
            else:
                self.exit_seen = 0

            if self.exit_seen >= self.EXIT_STREAK:
                self._set(EvacState.DONE)

        # state machine
        if self.state == EvacState.ENTER:
            if self._t() < self.ENTER_SECONDS:
                motors.run(self.BASE_FWD, self.BASE_FWD)
            else:
                self._set(EvacState.FOLLOW)

        elif self.state == EvacState.HIT_RECOVER:
            tt = self._t()

            # front hit -> reverse, back hit -> forward, else reverse
            drive_dir = -1.0 if self.front_hit(t) else (+1.0 if self.back_hit(t) else -1.0)

            if tt < self.HIT_BACK_SECONDS:
                motors.run(drive_dir * 0.35, drive_dir * 0.35)

            elif tt < (self.HIT_BACK_SECONDS + self.HIT_TURN_SECONDS):
                s = self.TURN_SPEED
                if self.turn_dir < 0:
                    motors.run(-s, +s)
                else:
                    motors.run(+s, -s)

            else:
                self._set(EvacState.FOLLOW)

        elif self.state == EvacState.TILT_RECOVER:
            tt = self._t()

            if tt < self.TILT_BACK_SECONDS:
                motors.run(-0.35, -0.35)

            elif tt < (self.TILT_BACK_SECONDS + self.TILT_TURN_SECONDS):
                s = self.TURN_SPEED
                if self.turn_dir < 0:
                    motors.run(-s, +s)
                else:
                    motors.run(+s, -s)

            else:
                self._set(EvacState.FOLLOW)

        elif self.state == EvacState.TURN:
            if self._t() < self.TURN_SECONDS:
                s = self.TURN_SPEED
                if self.turn_dir < 0:
                    motors.run(-s, +s)
                else:
                    motors.run(+s, -s)
            else:
                self._set(EvacState.FOLLOW)

        elif self.state == EvacState.FIND_EXIT:
            # placeholder: creep forward until exit detector hits
            motors.run(0.25, 0.25)

        elif self.state == EvacState.DONE:
            motors.run(0, 0)
            listener.set_mode(1)  # back to line mode
            return

        else:  # FOLLOW
            # If blocked ahead: choose turn direction based on which side is more open.
            if fwd < self.FRONT_STOP_MM:
                self.turn_dir = -1 if left > right else +1
                motors.run(0, 0)
                self._set(EvacState.TURN)
                return

            # If we lost the right wall: arc right to reacquire
            if right > self.RIGHT_LOST_MM:
                self.turn_dir = +1
                motors.run(self.BASE_FWD * 0.60, self.BASE_FWD * 0.25)
                return

            # Right wall follow P-control
            err = float(right - self.WALL_SETPOINT_MM)  # + = too far -> turn right
            turn = _clamp(self.KP_WALL * err, -self.TURN_MAX, self.TURN_MAX)

            left_spd = _clamp(self.BASE_FWD + turn)
            right_spd = _clamp(self.BASE_FWD - turn)
            motors.run(left_spd, right_spd)


_controller: EvacController | None = None
_started = False


def main(robot_state, evac_cam) -> None:
    """
    Called from main.py when mode == 2.
    main.py already starts both cameras in mode 2.
    """
    global _controller, _started

    # if mode changed away from evac, mark not started so next entry resets
    if listener.get_mode() != 2:
        _started = False
        return

    if _controller is None:
        _controller = EvacController()

    if not _started:
        _controller.reset()
        _started = True

    if hasattr(robot_state, "debug_text"):
        robot_state.debug_text.clear()

    led.on()

    evac_frame = None
    line_frame = None

    try:
        evac_frame = evac_cam.capture_array() if evac_cam is not None else None
    except Exception:
        evac_frame = None

    try:
        line_frame = line_camera.capture_array()
    except Exception:
        line_frame = None

    _controller.step(robot_state, evac_frame, line_frame)

    # footer debug
    if hasattr(robot_state, "debug_text"):
        health_tick(robot_state.debug_text)
        if getattr(robot_state, "debug", False):
            debug_lines(robot_state.debug_text)

    time.sleep(0.001)
