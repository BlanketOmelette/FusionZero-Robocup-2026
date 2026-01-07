class RobotState:
    def __init__(self):
        self._init_state()

    def _init_state(self):
        # Constants
        self.debug = True
        self.timings = True

        # Variables
        self.debug_text = []
        self.oled_put_text = None

        # Counters you will actually use
        self.count = {
            "main_loop": 0,
            "silver": 0,
            "red": 0,
            "touch": 0,
        }

        # Triggers (latched until you clear them)
        self.trigger = {
            "evacuation_zone": False,
        }

        # Silver detection debounce state
        self._silver_streak = 0
        self._silver_cooldown = 0

    def reset(self):
        self._init_state()

    def tick(self):
        """Call once per line loop iteration."""
        self.count["main_loop"] += 1
        if self._silver_cooldown > 0:
            self._silver_cooldown -= 1

    def update_silver(
        self,
        confidence: float,
        *,
        threshold: float = 0.85,
        consecutive: int = 3,
        cooldown_frames: int = 25,
    ) -> bool:
        """
        Feed per frame silver confidence here.
        If confidence is >= threshold for `consecutive` frames, we trigger evac.
        Returns True only on the frame it triggers.
        """
        if self._silver_cooldown > 0:
            self._silver_streak = 0
            return False

        if confidence >= threshold:
            self._silver_streak += 1
        else:
            self._silver_streak = 0

        if self._silver_streak >= consecutive:
            self._silver_streak = 0
            self._silver_cooldown = cooldown_frames
            self.count["silver"] += 1
            self.trigger["evacuation_zone"] = True
            return True

        return False

    def clear_evac_trigger(self):
        self.trigger["evacuation_zone"] = False

    def trigger_summary(self) -> str:
        evac = 1 if self.trigger.get("evacuation_zone") else 0
        return f"EVAC:{evac} SIL:{self.count['silver']}"
