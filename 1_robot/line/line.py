from core.shared_imports import time, cv2
from core.utilities import debug_lines, start_display, health_tick
from core.listener import listener
from hardware.robot import *

start_display()

def main(start_time, robot_state, line_follow, silver_detector=None) -> None:
    robot_state.debug_text.clear()
    overall_start = time.perf_counter()

    led.on()

    # STARTING PHASE
    if time.perf_counter() - start_time < 3:
        while (time.perf_counter() - start_time < 3) and listener.get_mode() != 0:
            time.sleep(0.001)
            motors.run(0, 0)
            found_black = line_follow.follow(starting=True)
            robot_state.tick()
            
        return

    # NORMAL LINE FOLLOW
    line_follow.follow()
    robot_state.tick()

    update_silver_trigger(line_follow, robot_state, silver_detector)

    # Act on trigger
    if robot_state.trigger.get("evacuation_zone", False):
        robot_state.clear_evac_trigger()
        motors.run(0, 0)
        listener.set_mode(2)
        return

    total_elapsed = time.perf_counter() - overall_start
    fps = int(1.0 / total_elapsed) if total_elapsed > 0 else 0

    robot_state.debug_text.append(f"FPS: {fps}")
    robot_state.debug_text.append(robot_state.trigger_summary())
    health_tick(robot_state.debug_text)

    if robot_state.debug:
        debug_lines(robot_state.debug_text)

    time.sleep(0.001)

def update_silver_trigger(line_follow, robot_state, silver_detector,
                          min_interval: float = 0.1,
                          hi_conf: float = 0.9) -> None:
    """
    Runs silver inference only when there is no top line.
    Rate limited to min_interval unless forced_next is set.
    Two consecutive high confidence (> hi_conf) silver hits triggers evacuation.
    """

    if silver_detector is None:
        return

    img = line_follow.image
    black_mask = line_follow.black_mask
    green = line_follow.green_signal

    # Need an image to infer
    if img is None:
        robot_state._silver_force_next = False
        robot_state._silver_hi_streak = 0
        return

    # "No top line" definition:
    # - if black_mask is None -> no top line
    # - else top H/10 band has zero pixels
    if black_mask is None or black_mask.size == 0:
        no_top = True
    else:
        h = black_mask.shape[0]
        band_h = max(1, h // 4)
        top_band = black_mask[:band_h, :]
        no_top = (cv2.countNonZero(top_band) == 0)

    want_check = (not green) and no_top

    # Init state if missing (no getattr)
    if not hasattr(robot_state, "_silver_last_t"):
        robot_state._silver_last_t = 0.0
    if not hasattr(robot_state, "_silver_force_next"):
        robot_state._silver_force_next = False
    if not hasattr(robot_state, "_silver_hi_streak"):
        robot_state._silver_hi_streak = 0

    # If we are not in the gap condition, reset silver logic
    if not want_check:
        robot_state._silver_force_next = False
        robot_state._silver_hi_streak = 0
        return

    now = time.perf_counter()
    due = (now - robot_state._silver_last_t) >= min_interval

    if not robot_state._silver_force_next and not due:
        return

    # Run inference
    robot_state._silver_force_next = False
    robot_state._silver_last_t = now

    result = silver_detector.predict(img)
    robot_state.debug_text.append(
        f"SILVER: {result['class_name']}, {result['confidence']:.3f}"
    )

    is_hi = (result["prediction"] == 1) and (result["confidence"] > hi_conf)

    if is_hi:
        robot_state._silver_hi_streak += 1

        if robot_state._silver_hi_streak >= 2:
            robot_state.trigger["evacuation_zone"] = True
            robot_state._silver_hi_streak = 0
            robot_state._silver_force_next = False
        else:
            # force immediate next loop inference
            robot_state._silver_force_next = True
    else:
        robot_state._silver_hi_streak = 0
        robot_state._silver_force_next = False
