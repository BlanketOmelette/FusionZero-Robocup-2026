from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import random

from core.shared_imports import time, np
from core.utilities import show, debug_lines, health_tick
from core.listener import listener
from hardware.robot import motors, lasers, touch, imu, line_camera, led


# ----------------------------
# tiny helpers
# ----------------------------
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


# ----------------------------
# mission and states
# ----------------------------
class MissionPhase(Enum):
    COLLECT_VICTIMS = auto()
    FIND_GREEN = auto()
    FIND_RED = auto()
    FIND_EXIT = auto()


class EvacState(Enum):
    ENTER_ALIGN = auto()
    ENTER_DRIVE = auto()

    SEARCH_SPIN_FAST = auto()
    SEARCH_SPIN_SLOW = auto()
    SEARCH_FORWARD = auto()

    APPROACH_TARGET = auto()
    GRAB = auto()
    DUMP = auto()

    TURN_AVOID = auto()
    HIT_RECOVER = auto()
    TILT_RECOVER = auto()

    DONE = auto()


@dataclass
class Detection:
    # kind examples: "victim_live", "victim_dead", "green", "red"
    kind: str
    bbox_area: float         # larger means closer / more confident target
    cx: float                # -1 .. +1 horizontal offset, left negative, right positive


# ----------------------------
# controller
# ----------------------------
class EvacController:
    # driving
    BASE_FWD = 0.32
    TURN_SPEED = 0.32
    SPIN_FAST_SPEED = 0.35
    SPIN_SLOW_SPEED = 0.22

    # thresholds
    FRONT_STOP_MM = 160
    TILT_LIMIT_DEG = 18

    # enter timing (tune)
    ENTER_ALIGN_SECONDS = 0.35
    ENTER_DEEP_SECONDS = 1.30

    # recovery timing (tune)
    HIT_BACK_SECONDS = 0.35
    HIT_TURN_SECONDS = 0.45
    TILT_BACK_SECONDS = 0.45
    TILT_TURN_SECONDS = 0.50

    # exit detection (still cheap)
    EXIT_BLACK_RATIO = 0.22
    EXIT_STREAK = 4

    def __init__(self) -> None:
        self.state: EvacState = EvacState.ENTER_ALIGN
        self.phase: MissionPhase = MissionPhase.COLLECT_VICTIMS

        self.t0 = time.perf_counter()
        self.state_timeout = 0.0  # seconds, 0 means "no timer"

        self.turn_dir = +1  # +1 right, -1 left
        self.exit_seen = 0

        # victim bookkeeping
        self.live_count = 0
        self.dead_count = 0

        # current target (victim/marker)
        self.target: Detection | None = None

        # optional: deterministic randomness while testing
        # random.seed(0)

    # ----------------------------
    # state timing helpers
    # ----------------------------
    def reset(self) -> None:
        self.state = EvacState.ENTER_ALIGN
        self.phase = MissionPhase.COLLECT_VICTIMS
        self.t0 = time.perf_counter()
        self.state_timeout = 0.0
        self.turn_dir = +1
        self.exit_seen = 0
        self.live_count = 0
        self.dead_count = 0
        self.target = None

    def _set(self, s: EvacState, timeout: float = 0.0) -> None:
        if s != self.state:
            self.state = s
            self.t0 = time.perf_counter()
            self.state_timeout = float(timeout)

    def _t(self) -> float:
        return time.perf_counter() - self.t0

    def _time_up(self) -> bool:
        return (self.state_timeout > 0.0) and (self._t() >= self.state_timeout)

    # ----------------------------
    # sensors
    # ----------------------------
    def read_tof(self) -> dict[str, int | None]:
        return lasers.read_dict()  # {"forward": mm|None, "left": mm|None, "right": mm|None}

    def read_touch(self) -> list[int]:
        return touch.read()  # [FL, FR, BL, BR]

    def read_imu(self) -> list[int]:
        ang = imu.read()
        return [0, 0, 0] if ang is None else ang

    # ----------------------------
    # safety checks
    # ----------------------------
    def any_hit(self, t: list[int]) -> bool:
        return any(_pressed(x) for x in t)

    def front_hit(self, t: list[int]) -> bool:
        return _pressed(t[0]) or _pressed(t[1])

    def back_hit(self, t: list[int]) -> bool:
        return _pressed(t[2]) or _pressed(t[3])

    def preferred_turn_from_hit(self, t: list[int]) -> int:
        fl, fr, bl, br = t
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

    # ----------------------------
    # dummy perception
    # replace these later with your models
    # ----------------------------
    def detect_victims(self, evac_frame) -> list[Detection]:
        # TODO: run victim model on evac_frame
        # Return detections with bbox_area and cx in -1..+1
        return []

    def detect_green_triangle(self, evac_frame) -> list[Detection]:
        # TODO: run green triangle detector
        return []

    def detect_red_triangle(self, evac_frame) -> list[Detection]:
        # TODO: run red triangle detector
        return []

    def detect_entry_silver_line(self, line_frame) -> bool:
        # TODO: run your silver entry detector on line_frame
        return False

    def detect_exit_black_line(self, line_frame) -> bool:
        # Cheap placeholder, same style as your old exit detector
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

    # ----------------------------
    # target selection logic
    # ----------------------------
    def _need_more_victims(self) -> bool:
        return (self.live_count < 2) or (self.dead_count < 1)

    def pick_best_needed(self, dets: list[Detection]) -> Detection | None:
        """
        Chooses the biggest bbox target that is still needed.
        Victim rules: total 3 victims, exactly 2 live and 1 dead.
        """
        if not dets:
            return None

        needed: list[Detection] = []
        for d in dets:
            if d.kind == "victim_live" and self.live_count < 2:
                needed.append(d)
            elif d.kind == "victim_dead" and self.dead_count < 1:
                needed.append(d)
            elif d.kind in ("green", "red"):
                needed.append(d)

        if not needed:
            return None
        return max(needed, key=lambda x: x.bbox_area)

    # ----------------------------
    # dummy actions
    # ----------------------------
    def approach_target_step(self, det: Detection, fwd_mm: int) -> bool:
        """
        One-step approach controller.
        Returns True when "close enough" to grab/dump.
        For now, this is a stub that just times out via state timer,
        but you can later use bbox_area or TOF to decide closeness.
        """
        # steer based on cx
        turn = _clamp(det.cx * 0.25, -0.25, 0.25)
        left = _clamp(self.BASE_FWD - turn)
        right = _clamp(self.BASE_FWD + turn)
        motors.run(left, right)

        # placeholder "close enough" check (replace later)
        return False

    def grab_victim(self, det: Detection) -> bool:
        """
        Dummy grab function. Replace with real grabber code.
        """
        # motors stopped for grab
        motors.run(0, 0)
        # TODO: actuate grabber, confirm success, etc.
        return True

    def dump_victims(self) -> bool:
        """
        Dummy dump function. Replace with real dump code.
        """
        motors.run(0, 0)
        # TODO: actuate dump, confirm success, etc.
        return True

    # ----------------------------
    # planning: randomized search timings
    # ----------------------------
    def plan_spin_fast_seconds(self) -> float:
        # 2 to 3 seconds fast spin
        return random.uniform(2.0, 3.0)

    def plan_spin_slow_seconds(self) -> float:
        # slower follow-up turn, roughly another full turn but slower
        return random.uniform(2.8, 4.2)

    def plan_forward_seconds(self) -> float:
        # random creep forward before next spin cycle
        return random.uniform(0.6, 1.2)

    # ----------------------------
    # main step
    # ----------------------------
    def step(self, robot_state, evac_frame, line_frame) -> None:
        # hard safety: only drive in evac mode
        if listener.get_mode() != 2:
            motors.run(0, 0)
            return

        tof = self.read_tof()
        t = self.read_touch()
        ang = self.read_imu()

        fwd = _mm(tof.get("forward"))
        left = _mm(tof.get("left"))
        right = _mm(tof.get("right"))

        # two separate panels
        if evac_frame is not None:
            show(evac_frame, name="evac_cam", display=True, debug_lines=None)
        if line_frame is not None:
            show(line_frame, name="line_cam", display=True, debug_lines=None)

        # bookkeeping
        if hasattr(robot_state, "tick"):
            robot_state.tick()

        if hasattr(robot_state, "debug_text"):
            robot_state.debug_text.append(f"EVAC state={self.state.name} phase={self.phase.name}")
            robot_state.debug_text.append(f"TOF F:{fwd} L:{left} R:{right}")
            robot_state.debug_text.append(f"TOUCH {t}")
            robot_state.debug_text.append(f"IMU {ang}")
            robot_state.debug_text.append(f"VICTIMS live={self.live_count} dead={self.dead_count}")

        # ----------------------------
        # overrides: tilt and touch
        # ----------------------------
        if self.tilt_bad(ang) and self.state not in (EvacState.TILT_RECOVER, EvacState.HIT_RECOVER):
            self._set(EvacState.TILT_RECOVER, timeout=self.TILT_BACK_SECONDS + self.TILT_TURN_SECONDS)

        if self.any_hit(t) and self.state not in (EvacState.HIT_RECOVER, EvacState.TILT_RECOVER):
            self.turn_dir = self.preferred_turn_from_hit(t)
            self._set(EvacState.HIT_RECOVER, timeout=self.HIT_BACK_SECONDS + self.HIT_TURN_SECONDS)

        # obstacle avoid (simple)
        if (
            fwd < self.FRONT_STOP_MM
            and self.state not in (EvacState.HIT_RECOVER, EvacState.TILT_RECOVER, EvacState.TURN_AVOID, EvacState.GRAB, EvacState.DUMP)
        ):
            # pick the more open side
            self.turn_dir = -1 if left > right else +1
            self._set(EvacState.TURN_AVOID, timeout=random.uniform(0.6, 0.9))

        # ----------------------------
        # mission transitions
        # ----------------------------
        if self.phase == MissionPhase.COLLECT_VICTIMS and not self._need_more_victims():
            self.phase = MissionPhase.FIND_GREEN
            self.target = None

        # exit detection only when we are in the exit phase
        if self.phase == MissionPhase.FIND_EXIT:
            if self.detect_exit_black_line(line_frame):
                self.exit_seen += 1
            else:
                self.exit_seen = 0

            if self.exit_seen >= self.EXIT_STREAK:
                self._set(EvacState.DONE)

        # ----------------------------
        # detection trigger (only during search states)
        # ----------------------------
        in_search = self.state in (EvacState.SEARCH_SPIN_FAST, EvacState.SEARCH_SPIN_SLOW, EvacState.SEARCH_FORWARD)

        if in_search and self.target is None:
            dets: list[Detection] = []

            if self.phase == MissionPhase.COLLECT_VICTIMS:
                dets = self.detect_victims(evac_frame)

            elif self.phase == MissionPhase.FIND_GREEN:
                # tag them as "green"
                dets = self.detect_green_triangle(evac_frame)
                for d in dets:
                    d.kind = "green"

            elif self.phase == MissionPhase.FIND_RED:
                dets = self.detect_red_triangle(evac_frame)
                for d in dets:
                    d.kind = "red"

            # choose the biggest bbox that is still needed
            best = self.pick_best_needed(dets)
            if best is not None:
                self.target = best
                self._set(EvacState.APPROACH_TARGET, timeout=2.5)  # timeout safety, tune later

        # ----------------------------
        # state machine
        # ----------------------------
        if self.state == EvacState.ENTER_ALIGN:
            # choose direction based on which side is more open
            self.turn_dir = -1 if left > right else +1

            # turn on spot a little
            s = 0.28
            if self.turn_dir < 0:
                motors.run(-s, +s)
            else:
                motors.run(+s, -s)

            if self._t() >= self.ENTER_ALIGN_SECONDS:
                self._set(EvacState.ENTER_DRIVE, timeout=self.ENTER_DEEP_SECONDS)

        elif self.state == EvacState.ENTER_DRIVE:
            # drive "deep" into evac zone
            motors.run(self.BASE_FWD, self.BASE_FWD)
            if self._time_up():
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())

        elif self.state == EvacState.SEARCH_SPIN_FAST:
            s = self.SPIN_FAST_SPEED
            motors.run(+s, -s)  # default spin right
            if self._time_up():
                self._set(EvacState.SEARCH_SPIN_SLOW, timeout=self.plan_spin_slow_seconds())

        elif self.state == EvacState.SEARCH_SPIN_SLOW:
            s = self.SPIN_SLOW_SPEED
            motors.run(+s, -s)
            if self._time_up():
                self._set(EvacState.SEARCH_FORWARD, timeout=self.plan_forward_seconds())

        elif self.state == EvacState.SEARCH_FORWARD:
            motors.run(self.BASE_FWD, self.BASE_FWD)
            if self._time_up():
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())

        elif self.state == EvacState.TURN_AVOID:
            s = self.TURN_SPEED
            if self.turn_dir < 0:
                motors.run(-s, +s)
            else:
                motors.run(+s, -s)
            if self._time_up():
                # go back to search loop
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())

        elif self.state == EvacState.APPROACH_TARGET:
            if self.target is None:
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())
            else:
                reached = self.approach_target_step(self.target, fwd_mm=fwd)
                if reached:
                    self._set(EvacState.GRAB, timeout=1.0)
                elif self._time_up():
                    # gave up, go back to searching
                    self.target = None
                    self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())

        elif self.state == EvacState.GRAB:
            # for now, always "succeeds" if we had a target
            if self.target is not None and self.grab_victim(self.target):
                # update mission counters or phase
                if self.target.kind == "victim_live" and self.live_count < 2:
                    self.live_count += 1
                elif self.target.kind == "victim_dead" and self.dead_count < 1:
                    self.dead_count += 1
                elif self.target.kind == "green":
                    # green found, dump now
                    self._set(EvacState.DUMP, timeout=1.0)
                    return
                elif self.target.kind == "red":
                    # red found, now find exit
                    self.phase = MissionPhase.FIND_EXIT

                self.target = None

                # after each grab attempt, resume search
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())
            else:
                # nothing to grab
                self.target = None
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())

        elif self.state == EvacState.DUMP:
            if self.dump_victims():
                # after dumping at green, look for red triangle
                self.phase = MissionPhase.FIND_RED
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())
            elif self._time_up():
                # fail safe
                self.phase = MissionPhase.FIND_RED
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())

        elif self.state == EvacState.HIT_RECOVER:
            tt = self._t()
            drive_dir = -1.0 if self.front_hit(t) else (+1.0 if self.back_hit(t) else -1.0)

            if tt < self.HIT_BACK_SECONDS:
                motors.run(drive_dir * 0.35, drive_dir * 0.35)
            else:
                s = self.TURN_SPEED
                if self.turn_dir < 0:
                    motors.run(-s, +s)
                else:
                    motors.run(+s, -s)

            if self._time_up():
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())

        elif self.state == EvacState.TILT_RECOVER:
            tt = self._t()

            if tt < self.TILT_BACK_SECONDS:
                motors.run(-0.35, -0.35)
            else:
                s = self.TURN_SPEED
                if self.turn_dir < 0:
                    motors.run(-s, +s)
                else:
                    motors.run(+s, -s)

            if self._time_up():
                self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())

        elif self.state == EvacState.DONE:
            motors.run(0, 0)
            listener.set_mode(1)
            return

        else:
            # default safety
            motors.run(0, 0)
            self._set(EvacState.SEARCH_SPIN_FAST, timeout=self.plan_spin_fast_seconds())


_controller: EvacController | None = None
_started = False


def main(robot_state, evac_cam) -> None:
    """
    Called from main.py when mode == 2.
    """
    global _controller, _started

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

    # capture frames
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
