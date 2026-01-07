from core.shared_imports import time, random, os

from core.listener import listener
from core.utilities import *

import line.line as line
import line.evacuation as evac

from hardware.robot import *

from line.robot_state import RobotState
from line.line_follower import LineFollower

import os

record = True

robot_state = RobotState()
line_follow = LineFollower(robot_state)

# Optional: load silver model once (won't crash if file/module missing)
SILVER_MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "silver_detector.ts")
silver_detector = None

try:
    if os.path.exists(SILVER_MODEL_PATH):
        from line.silver_detector import SilverLineDetector
        silver_detector = SilverLineDetector(SILVER_MODEL_PATH, device="cpu")
        print("Silver model loaded:", SILVER_MODEL_PATH)
    else:
        print("Silver model not found:", SILVER_MODEL_PATH)
except Exception as e:
    print("Silver model disabled:", e)
    silver_detector = None

configure_finalize_gesture(
    get_mode=listener.get_mode,
    read_touches=touch.read,
    led_blink=led.blink,
    required_mode=0,
    hold_seconds=3.0,
    pressed_value=1,
    action="interrupt",
)

def main() -> None:
    motors.run(0, 0)
    led.off()

    start_time = time.perf_counter()
    random.seed(time.time())

    start_display()

    last_mode = None

    try:
        listener.start()
        oled.display_logo()

        while not listener.exit_event.is_set():
            gesture_tick()
            mode = listener.get_mode()

            if mode != last_mode:
                if mode != 1:
                    start_time = time.perf_counter()

                if mode == 0:
                    motors.run(0, 0)
                    led.off()
                    robot_state.reset()

                    line_camera.stop()
                    evac_camera.stop()

                elif mode == 1:
                    line_camera.start()
                    evac_camera.stop()

                elif mode == 2:
                    evac_camera.start()
                    line_camera.start()

                else:
                    line_camera.stop()
                    evac_camera.stop()

                last_mode = mode

            # Run the selected mode
            if mode == 0:
                pass

            elif mode == 1:
                line.main(start_time, robot_state, line_follow, silver_detector)

            elif mode == 2:
                evac.main(robot_state, evac_camera)

            elif mode == 3:
                led.on()

                imu_values = imu.read()
                imu_values = imu_values if imu_values is not None else ""

                debug([
                    "READING",
                    " ".join(map(str, touch.read())),
                    " ".join(map(str, lasers.read())),
                    " ".join(map(str, imu_values)),
                ], [12, 9, 30, 6, 20, 11])

            elif mode == 9:
                listener.exit_event.set()

    except KeyboardInterrupt:
        print("Exiting gracefully!")

    finally:
        oled.clear()
        oled.text("EXITING", 0, 0)

        motors.run(0, 0)
        oled.text("Motor ✓", 0, 15)

        led.close()
        oled.text("LED ✓", 0, 30)

        if record:
            oled.text("SAVE ...", 0, 45)
            try:
                print(f"Saved: {get_session_dir()}")
                oled.text("Save ✓", 0, 45)
            except Exception:
                print("Frames saved to session folder.")
                oled.text("Save ✗", 0, 45)

        stop_display()
        oled.text("Disp ✓", 70, 0)

        line_camera.close()
        oled.text("LineCam ✓", 70, 15)

        evac_camera.close()
        oled.text("EvacCam ✓", 70, 30)

        listener.stop()

        restore_terminal()

        time.sleep(0.1)

def restore_terminal():
    os.system("stty sane")

if __name__ == "__main__":
    main()