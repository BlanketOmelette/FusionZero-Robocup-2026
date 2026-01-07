from core.shared_imports import time
from core.utilities import debug_lines, start_display, health_tick
from core.listener import listener
from hardware.robot import *

start_display()

def main(start_time, robot_state, line_follow, silver_detector=None) -> None:
    robot_state.debug_text.clear()
    overall_start = time.perf_counter()

    led.on()

    # If evac was already triggered previously, switch right away
    if robot_state.trigger.get("evacuation_zone", False):
        robot_state.clear_evac_trigger()
        motors.run(0, 0)
        listener.set_mode(2)
        return

    # STARTING PHASE (no silver detection here)
    if time.perf_counter() - start_time < 3:
        while (time.perf_counter() - start_time < 3) and listener.get_mode() != 0:
            time.sleep(0.001)
            motors.run(0, 0)

            found_black = line_follow.follow(starting=True)
            robot_state.tick()

            if found_black:
                start_time = time.perf_counter() - 3
                break

        return  # keep it simple, no extra logic in starting

    # NORMAL LINE FOLLOW
    line_follow.follow()
    robot_state.tick()

    # Silver detection (inline, minimal gates)
    if silver_detector is not None:
        img = getattr(line_follow, "image", None)
        black_mask = getattr(line_follow, "black_mask", None)
        green = bool(getattr(line_follow, "green_signal", False))

        # basic gates: only run ML when we have valid masks and not seeing green
        if img is not None and black_mask is not None and not green:
            # optional ROI crop: bottom half tends to be most relevant
            h = img.shape[0]
            roi = img[h // 2 :, :]

            result = silver_detector.predict(roi)
            robot_state.debug_text.append(
                f"SILVER: {result['class_name']}, {result['confidence']:.3f}"
            )

            # streak counter then latch trigger
            if result["prediction"] == 1 and result["confidence"] > 0.98:
                robot_state.count["silver"] = robot_state.count.get("silver", 0) + 1
            else:
                robot_state.count["silver"] = 0

            if robot_state.count["silver"] >= 3:
                robot_state.trigger["evacuation_zone"] = True
                robot_state.count["silver"] = 0

    # Act on trigger immediately
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
