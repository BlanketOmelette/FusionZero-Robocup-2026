from core.shared_imports import threading, Queue, Empty, cv2, np, time, os

# =========================
# Session / Paths (SD card)
# =========================

SESSION_ROOT = "sessions"  # parent folder on SD (relative to cwd)
SESSION_DIR = None         # set by _init_session()
META_PATH = None           # timestamps.csv path
_meta_file = None          # csv file handle
_session_inited = False

def _init_session():
    """
    Create a new session folder like sessions/2025-10-04_19-42-31/
    and a timestamps.csv file inside it.
    """
    global SESSION_DIR, META_PATH, _meta_file, _session_inited

    if _session_inited:
        return

    os.makedirs(SESSION_ROOT, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
    SESSION_DIR = os.path.join(SESSION_ROOT, stamp)
    os.makedirs(SESSION_DIR, exist_ok=True)

    META_PATH = os.path.join(SESSION_DIR, "timestamps.csv")
    _meta_file = open(META_PATH, "w", buffering=1)  # line-buffered
    _meta_file.write("timestamp,filename\n")
    _session_inited = True

def _close_session():
    global _meta_file
    try:
        if _meta_file is not None:
            _meta_file.flush()
            _meta_file.close()
    except Exception:
        pass
    _meta_file = None


# =========================
# Pi Health (CPU/MEM/TEMP)
# =========================

_health_state = {
    "last_timestamp": 0.0,
    "prev_idle": None,
    "prev_total": None,
    "cpu": None,
    "ram": None,
    "temp": None,
}

def read_cpu_percent() -> float:
    with open("/proc/stat", "r") as f:
        fields = f.readline().split()[1:8]
    user, nice, system, idle, iowait, irq, softirq = map(int, fields)
    idle_all = idle + iowait
    non_idle = user + nice + system + irq + softirq
    total = idle_all + non_idle

    prev_idle = _health_state["prev_idle"]
    prev_total = _health_state["prev_total"]
    _health_state["prev_idle"] = idle_all
    _health_state["prev_total"] = total

    if prev_idle is None or prev_total is None:
        return 0.0

    delta_idle = idle_all - prev_idle
    delta_total = total - prev_total
    if delta_total <= 0:
        return 0.0

    return int(100 * (1.0 - (delta_idle / delta_total)))

def read_mem_used_total_mb() -> tuple[int, int]:
    mem_total_kb = None
    mem_available_kb = None
    with open("/proc/meminfo", "r") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                mem_total_kb = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                mem_available_kb = int(line.split()[1])
            if mem_total_kb is not None and mem_available_kb is not None:
                break
    used_mb = (mem_total_kb - mem_available_kb) // 1024
    total_mb = mem_total_kb // 1024
    used_gb  = used_mb / 1024.0
    total_gb = total_mb / 1024.0
    return used_gb, total_gb

def read_temp_celsius() -> float:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return int(f.read()) // 1000
    except Exception:
        return 0.0

def health_tick(debug_list: list[str], every_seconds: float = 0.5) -> None:
    """
    Append a health line every call. If less than `every_seconds` since last
    update, repeat the previous reading instead of skipping.
    """
    now = time.perf_counter()
    # compute a fresh reading if it's time OR if we have none yet
    if _health_state["cpu"] is None or (now - _health_state["last_timestamp"] >= every_seconds):
        _health_state["last_timestamp"] = now
        _health_state["cpu"] = read_cpu_percent()
        _health_state["ram"] = read_mem_used_total_mb()
        _health_state["temp"] = read_temp_celsius()

    # always append the cached line
    debug_list.append(f"CPU: {_health_state['cpu']}%")
    debug_list.append(f"RAM: {_health_state['ram'][0]:0.1f}/{_health_state['ram'][1]:0.1f}GB")
    debug_list.append(f"TEMP: {_health_state['temp']}°C")


# =========================
# Printing / Display
# =========================

# Fixed widths for debug lines: [TRIGGERS, GREEN, TURN, INT, FPS, CPU, MEM, TEMP]
DEBUG_FIXED_WIDTHS = [12, 23, 13, 15, 13, 14, 20, 14]

# Print Functions
def debug(data: list[str], coloumn_widths: list[int], separator: str = "|") -> None:
    formatted_cells = [f"{cell:^{width}}" for cell, width in zip(data, coloumn_widths)]
    print(f" {separator} ".join(formatted_cells))

def debug_lines(entries: list[str], padding: int = 2, separator: str = "|") -> None:
    """
    Fixed-only renderer:
      - First N columns use DEBUG_FIXED_WIDTHS.
      - Extra columns auto-size to len(cell) + padding.
    """
    if not entries:
        return

    n_fixed = len(DEBUG_FIXED_WIDTHS)
    if len(entries) <= n_fixed:
        col_widths = DEBUG_FIXED_WIDTHS[:len(entries)]
    else:
        fixed = DEBUG_FIXED_WIDTHS[:]
        extras = entries[n_fixed:]
        extra_widths = [max(1, len(str(e)) + padding) for e in extras]
        col_widths = fixed + extra_widths

    formatted = [f"{str(cell):^{width}}" for cell, width in zip(entries, col_widths)]
    print(f" {separator} ".join(formatted))


# =========================
# Display & Saving (to SD)
# =========================

# You can override these from main if you want
DISPLAY_ENABLED = True
SAVE_ENABLED = True

_display_thread = None
_display_queue: Queue = None
_save_thread = None
_save_queue: Queue = None
_stop_evt = threading.Event()

SAVE_EVERY_N = 5          # keep your old default
DISPLAY_QUEUE_MAX = 1     # keep only latest frame
SAVE_QUEUE_MAX = 2        # small buffer, drops if slow
_save_i = 0

# Keep a small in-RAM index of what we saved: list of (filepath, timestamp)
_saved_frames = []


def _display_worker(q: Queue):
    # Note: OpenCV imshow usually works fine in a thread on Pi
    # If you ever get weird UI issues, switch back to process, but try this first.
    while not _stop_evt.is_set():
        try:
            item = q.get(timeout=0.1)
        except Empty:
            continue

        if item is None:
            break

        frame, window_name = item
        if isinstance(frame, np.ndarray):
            cv2.imshow(window_name, frame)
            cv2.waitKey(1)

    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


def _save_worker(q: Queue):
    global _meta_file
    while not _stop_evt.is_set():
        try:
            item = q.get(timeout=0.1)
        except Empty:
            continue

        if item is None:
            break

        frame, ts = item
        try:
            fname = f"f_{int(ts*1e6):016d}.npy"
            fpath = os.path.join(SESSION_DIR, fname)
            np.save(fpath, frame)

            _saved_frames.append((fpath, ts))

            if _meta_file is not None:
                _meta_file.write(f"{ts:.6f},{fname}\n")
        except Exception:
            # Never let saving crash the robot
            pass


def get_session_dir():
    return SESSION_DIR


def start_display():
    global _display_thread, _display_queue, _save_thread, _save_queue
    _init_session()

    _stop_evt.clear()

    if _display_thread is None and DISPLAY_ENABLED:
        _display_queue = Queue(maxsize=DISPLAY_QUEUE_MAX)
        _display_thread = threading.Thread(target=_display_worker, args=(_display_queue,), daemon=True)
        _display_thread.start()

    if _save_thread is None and SAVE_ENABLED:
        _save_queue = Queue(maxsize=SAVE_QUEUE_MAX)
        _save_thread = threading.Thread(target=_save_worker, args=(_save_queue,), daemon=True)
        _save_thread.start()


def put_text_on_image(image, debug_lines: list[str]):
    origin = [10, 20]
    line_height = 20
    font_scale = 0.6
    color = (0, 0, 255)
    thickness = 1
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, line in enumerate(debug_lines):
        y = origin[1] + i * line_height
        cv2.putText(image, line, (origin[0], y), font, font_scale, color, thickness, cv2.LINE_AA)


def show(frame: np.ndarray, name: str = "Display", display: bool = True, debug_lines: list[str] = None):
    global _display_queue, _save_queue, _save_i

    _init_session()

    if debug_lines is not None:
        put_text_on_image(frame, debug_lines)

    # avoid copies when already uint8
    if frame.dtype != np.uint8:
        frame = frame.astype(np.uint8, copy=False)

    # --- async display (keeps only latest) ---
    if DISPLAY_ENABLED and display and _display_queue is not None:
        # drop stale frames
        while True:
            try:
                _display_queue.get_nowait()
            except Exception:
                break
        try:
            _display_queue.put_nowait((frame, name))
        except Exception:
            pass

    # --- async saving (keeps your SAVE_EVERY_N behavior) ---
    if SAVE_ENABLED and _save_queue is not None:
        _save_i = (_save_i + 1) % SAVE_EVERY_N
        if _save_i == 0:
            ts = time.perf_counter()
            # push to saver thread, drop if queue full
            try:
                _save_queue.put_nowait((frame.copy(), ts))
            except Exception:
                pass


def stop_display():
    global _display_thread, _display_queue, _save_thread, _save_queue

    _stop_evt.set()

    if _display_queue is not None:
        try:
            _display_queue.put_nowait(None)
        except Exception:
            pass

    if _save_queue is not None:
        try:
            _save_queue.put_nowait(None)
        except Exception:
            pass

    if _display_thread is not None:
        _display_thread.join(timeout=1.0)
    if _save_thread is not None:
        _save_thread.join(timeout=1.0)

    _display_thread = None
    _display_queue = None
    _save_thread = None
    _save_queue = None

    _close_session()


def get_saved_frames():
    return _saved_frames