# 1_robot/line/robot_state.py

class RobotState:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        # Debug
        self.debug = True
        self.timings = True
        self.debug_text = []
        self.oled_put_text = None

        # Counters
        self.count = {
            "main_loop": 0,
            "silver": 0,
        }

        # Triggers (latched flags)
        self.trigger = {
            "evacuation_zone": False,
        }

    def tick(self) -> None:
        """Call once per control loop iteration (or inside tight start loops)."""
        self.count["main_loop"] += 1

    def clear_evac_trigger(self) -> None:
        self.trigger["evacuation_zone"] = False

    def trigger_summary(self) -> str:
        evac = 1 if self.trigger.get("evacuation_zone", False) else 0
        sil = int(self.count.get("silver", 0))
        return f"EVAC:{evac} SIL:{sil}"