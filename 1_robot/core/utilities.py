from core.shared_imports import threading, Queue, Empty, cv2, np, time, os, csv, shutil, subprocess, Path

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

DISPLAY_ENABLED = True
SAVE_ENABLED = True

# multi window latest frame store
_display_latest = {}
_display_latest_lock = threading.Lock()
_display_latest_event = threading.Event()

_display_thread = None
_save_thread = None
_save_queue: Queue = None
_stop_evt = threading.Event()

SAVE_EVERY_N = 5
SAVE_QUEUE_MAX = 2
_save_i = 0

_saved_frames = []  # list of (filepath, timestamp)

# Video finalize defaults (drop in behavior)
FINALIZE_VIDEO_ON_STOP = True
FINALIZE_CLEANUP_ON_SUCCESS = True
FINALIZE_OUT_NAME = "run_vfr.mp4"

# Gesture finalize (optional, configured from main)
_gesture_cfg = {
    "enabled": False,
    "get_mode": None,         # callable -> int
    "read_touches": None,     # callable -> list/tuple of length touch_count
    "led_blink": None,        # callable(seconds: float) -> None
    "required_mode": 0,
    "touch_count": 4,
    "hold_seconds": 3.0,
    "check_every": 0.03,      # seconds
    "pressed_value": 1,
    "action": "interrupt",    # "interrupt" or "flag"
}

_gesture_state = {
    "t0": None,
    "armed": True,
    "last_check": 0.0,
}

_finalize_requested = False


def configure_finalize_gesture(
    *,
    get_mode,
    read_touches,
    led_blink=None,
    required_mode: int = 0,
    touch_count: int = 4,
    hold_seconds: float = 3.0,
    pressed_value: int = 1,
    action: str = "interrupt",
) -> None:
    """
    Call this once in your main after you create listener/touch/led.

    get_mode:     function returning current mode (int)
    read_touches: function returning iterable of touch states
    led_blink:    optional function(seconds) for feedback
    action:
      - "interrupt": raises KeyboardInterrupt from show() when held long enough
      - "flag": sets finalize_requested flag, your main loop can break manually
    """
    _gesture_cfg["enabled"] = True
    _gesture_cfg["get_mode"] = get_mode
    _gesture_cfg["read_touches"] = read_touches
    _gesture_cfg["led_blink"] = led_blink
    _gesture_cfg["required_mode"] = required_mode
    _gesture_cfg["touch_count"] = touch_count
    _gesture_cfg["hold_seconds"] = hold_seconds
    _gesture_cfg["pressed_value"] = pressed_value
    _gesture_cfg["action"] = action


def finalize_requested() -> bool:
    return bool(_finalize_requested)


def _touch_pressed(v) -> bool:
    # bool sensors
    if isinstance(v, bool):
        return v
    # int sensors
    if isinstance(v, int):
        return v == _gesture_cfg["pressed_value"]
    # fallback
    return bool(v)

def gesture_tick():
    _check_finalize_gesture()

def _check_finalize_gesture() -> None:
    """
    Called from show() at a low rate.
    If triggered:
      - blink LED (if provided)
      - set flag
      - optionally raise KeyboardInterrupt to emulate Ctrl+C
    """
    global _finalize_requested

    if not _gesture_cfg["enabled"]:
        return

    now = time.perf_counter()
    if (now - _gesture_state["last_check"]) < _gesture_cfg["check_every"]:
        return
    _gesture_state["last_check"] = now

    get_mode = _gesture_cfg["get_mode"]
    read_touches = _gesture_cfg["read_touches"]
    if get_mode is None or read_touches is None:
        return

    try:
        mode = int(get_mode())
    except Exception:
        return

    if mode != _gesture_cfg["required_mode"]:
        _gesture_state["t0"] = None
        _gesture_state["armed"] = True
        return

    try:
        touches = list(read_touches())
    except Exception:
        return

    if len(touches) < _gesture_cfg["touch_count"]:
        return

    all_pressed = True
    for i in range(_gesture_cfg["touch_count"]):
        if not _touch_pressed(touches[i]):
            all_pressed = False
            break

    if all_pressed:
        if _gesture_state["t0"] is None:
            _gesture_state["t0"] = now

        if _gesture_state["armed"] and (now - _gesture_state["t0"]) >= _gesture_cfg["hold_seconds"]:
            _gesture_state["armed"] = False
            _finalize_requested = True

            # LED feedback: quick triple blink
            blink = _gesture_cfg["led_blink"]
            if callable(blink):
                try:
                    for _ in range(3):
                        blink(0.08)
                except Exception:
                    pass

            if _gesture_cfg["action"] == "interrupt":
                raise KeyboardInterrupt
    else:
        _gesture_state["t0"] = None
        _gesture_state["armed"] = True


def _display_worker():
    while not _stop_evt.is_set():
        # wait until something new arrives, or wake periodically
        _display_latest_event.wait(timeout=0.05)
        _display_latest_event.clear()

        with _display_latest_lock:
            items = list(_display_latest.items())

        # show all windows that have frames
        for window_name, frame in items:
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
            pass


def get_session_dir():
    return SESSION_DIR

def start_display():
    global _display_thread, _save_thread, _save_queue
    _init_session()

    _stop_evt.clear()

    if _display_thread is None and DISPLAY_ENABLED:
        _display_thread = threading.Thread(target=_display_worker, daemon=True)
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
    global _save_queue, _save_i

    _init_session()

    if debug_lines is not None:
        put_text_on_image(frame, debug_lines)

    if frame.dtype != np.uint8:
        frame = frame.astype(np.uint8, copy=False)

    # gesture check (can raise KeyboardInterrupt)
    _check_finalize_gesture()

        # async display (latest per window)
    if DISPLAY_ENABLED and display:
        with _display_latest_lock:
            _display_latest[name] = frame
        _display_latest_event.set()

    # async saving
    if SAVE_ENABLED and _save_queue is not None:
        _save_i = (_save_i + 1) % SAVE_EVERY_N
        if _save_i == 0:
            ts = time.perf_counter()
            try:
                _save_queue.put_nowait((frame.copy(), ts))
            except Exception:
                pass

def _make_vfr_video_from_session(session_dir: str, out_name: str) -> str | None:
    sdir = Path(session_dir)
    meta = sdir / "timestamps.csv"
    if not meta.exists():
        return None

    if shutil.which("ffmpeg") is None:
        print("[video] ffmpeg not found, skipping mp4")
        return None

    rows: list[tuple[float, str]] = []
    with meta.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                ts = float(r["timestamp"])
                fname = str(r["filename"])
            except Exception:
                continue
            if (sdir / fname).exists():
                rows.append((ts, fname))

    if len(rows) < 2:
        print("[video] not enough frames to build video")
        return None

    rows.sort(key=lambda x: x[0])

    tmp_dir = sdir / "__tmp_png"
    tmp_dir.mkdir(exist_ok=True)
    concat_path = sdir / "__vfr_concat.txt"

    # Write PNGs
    png_rel_paths: list[str] = []
    for i, (ts, fname) in enumerate(rows):
        npy_path = sdir / fname
        frame = np.load(str(npy_path), allow_pickle=False)
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8, copy=False)
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        png_path = tmp_dir / f"frame_{i:06d}.png"
        cv2.imwrite(str(png_path), frame)

        # IMPORTANT: store path relative to session dir
        png_rel_paths.append((png_path.relative_to(sdir)).as_posix())

    # Write concat file using relative paths
    with concat_path.open("w") as cf:
        for i, rel in enumerate(png_rel_paths):
            cf.write(f"file '{rel}'\n")
            if i < len(rows) - 1:
                dt = max(0.0001, rows[i + 1][0] - rows[i][0])
                cf.write(f"duration {dt:.6f}\n")
        # repeat last frame so ffmpeg keeps it
        cf.write(f"file '{png_rel_paths[-1]}'\n")

    out_path = sdir / out_name
    if out_path.exists():
        stem, ext = out_path.stem, out_path.suffix
        k = 0
        while True:
            candidate = sdir / f"{stem}_{k}{ext}"
            if not candidate.exists():
                out_path = candidate
                break
            k += 1

    cmd = [
        "ffmpeg", "-y",
        "-loglevel", "error",
        "-f", "concat", "-safe", "0",
        "-i", concat_path.name,      # run from inside session dir
        "-vsync", "vfr",
        "-pix_fmt", "yuv420p",
        "-c:v", "libx264",
        "-crf", "18",
        out_path.name,
    ]

    try:
        subprocess.run(cmd, check=True, cwd=str(sdir))
    except Exception as e:
        print(f"[video] ffmpeg failed: {e}")
        return None
    finally:
        try:
            concat_path.unlink(missing_ok=True)
        except Exception:
            pass
        try:
            shutil.rmtree(tmp_dir, ignore_errors=True)
        except Exception:
            pass

    return str(out_path)


def _cleanup_session_frames_and_meta(session_dir: str) -> None:
    sdir = Path(session_dir)
    meta = sdir / "timestamps.csv"
    if meta.exists():
        try:
            with meta.open("r", newline="") as f:
                reader = csv.DictReader(f)
                for r in reader:
                    fname = r.get("filename")
                    if not fname:
                        continue
                    try:
                        (sdir / fname).unlink(missing_ok=True)
                    except Exception:
                        pass
        except Exception:
            pass

    try:
        meta.unlink(missing_ok=True)
    except Exception:
        pass


def stop_display(
    finalize_video: bool = FINALIZE_VIDEO_ON_STOP,
    out_name: str = FINALIZE_OUT_NAME,
    cleanup_on_success: bool = FINALIZE_CLEANUP_ON_SUCCESS,
):
    global _display_thread, _save_thread, _save_queue, _display_latest_event

    # Stop signal + wake display thread (so it can exit immediately)
    _stop_evt.set()
    _display_latest_event.set()  # wake display thread

    # Keep save queue shutdown signal (unchanged)
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
    _save_thread = None
    _save_queue = None

    _close_session()

    with _display_latest_lock: _display_latest.clear()   
    _display_latest_event.clear()
    _saved_frames.clear()

    # Finalize to MP4 after threads stop and timestamps flushed
    if finalize_video and SESSION_DIR is not None:
        out = _make_vfr_video_from_session(SESSION_DIR, out_name=out_name)
        if out and cleanup_on_success:
            _cleanup_session_frames_and_meta(SESSION_DIR)

            # LED feedback: one longer blink if configured
            blink = _gesture_cfg.get("led_blink")
            if callable(blink):
                try:
                    blink(0.25)
                except Exception:
                    pass

def get_saved_frames():
    return _saved_frames