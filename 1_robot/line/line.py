from core.shared_imports import time
from core.utilities import debug_lines, start_display,health_tick
from core.listener import listener
from hardware.robot import *

start_display()

def main(start_time, robot_state, line_follow) -> None:
    robot_state.debug_text.clear()
    overall_start = time.perf_counter()

    led.on()

    if time.perf_counter() - start_time < 3:
        while (time.perf_counter() - start_time < 3) and listener.get_mode() != 0:
            time.sleep(0.001)
            motors.run(0, 0)
            line_follow.follow(starting=True)

    else: line_follow.follow()

    total_elapsed = time.perf_counter() - overall_start
    fps = int(1.0 / total_elapsed) if total_elapsed > 0 else 0
    robot_state.debug_text.append(f"FPS: {fps}")
    health_tick(robot_state.debug_text)

    if robot_state.debug: debug_lines(robot_state.debug_text)

    time.sleep(0.001)
