from core.shared_imports import time, threading, Queue, Empty, Button


class ModeListener:
    def __init__(self, initial_mode: int = 0, button_pin: int = 27):
        self.button_pin = button_pin

        self.mode = initial_mode
        self._mode_lock = threading.Lock()

        self.exit_event = threading.Event()
        self.input_queue: Queue[str] = Queue()

        self._threads = []
        self._button: Button | None = None

    def start(self) -> None:
        self.exit_event.clear()

        t_in = threading.Thread(target=self._input_thread, daemon=True)
        t_cmd = threading.Thread(target=self._command_thread, daemon=True)
        t_btn = threading.Thread(target=self._button_thread, daemon=True)

        self._threads = [t_in, t_cmd, t_btn]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self.exit_event.set()
        # no join needed because daemon threads will exit, but you can join if you want

    def set_mode(self, m: int) -> None:
        with self._mode_lock:
            self.mode = int(m)

    def get_mode(self) -> int:
        with self._mode_lock:
            return int(self.mode)

    # ---------- threads ----------

    def _input_thread(self) -> None:
        # blocks on stdin, pushes raw strings into queue
        while not self.exit_event.is_set():
            try:
                s = input().strip()
            except Exception:
                return
            self.input_queue.put(s)

    def _command_thread(self) -> None:
        valid = {"0", "1", "2", "3", "4", "5", "9"}

        while not self.exit_event.is_set():
            try:
                s = self.input_queue.get(timeout=0.1)
            except Empty:
                continue

            if s not in valid:
                # match your old behavior: default to 1 if invalid
                self.set_mode(1)
                continue

            m = int(s)
            self.set_mode(m)
            if m == 9:
                self.exit_event.set()

    def _button_thread(self) -> None:
        # gpiozero handles pull-ups and Pi 5 GPIO backend properly
        self._button = Button(self.button_pin, pull_up=True, bounce_time=0.02)

        prev_pressed = self._button.is_pressed

        while not self.exit_event.is_set():
            pressed = self._button.is_pressed

            if pressed != prev_pressed:
                # only change mode when the switch actually changes
                self.set_mode(1 if pressed else 0)
                prev_pressed = pressed

            time.sleep(0.02)

        try:
            self._button.close()
        except Exception:
            pass

            
listener = ModeListener()