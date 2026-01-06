from core.shared_imports import time, cv2, np, deque
from core.utilities import show
from hardware.robot import *
from core.listener import listener

class LineFollower:
    # ======================================================================
    # INIT
    # ======================================================================

    def __init__(self, robot_state):
        self.robot_state = robot_state

        # Follower
        self.straight_speed = 90          # max speed when error is near 0
        self.corner_speed = 60            # speed when error is large
        self.speed_ramp_error = 10        # abs(error) >= this means full corner_speed
        self.speed_approach = 60
        self.proportional_gain = 5

        # Smoothing
        self.error_window_size = 5
        self.error_history = deque(maxlen=self.error_window_size)

        # Thresholding
        self.min_black_area = 5000
        self.base_black = 110
        self.bright_black = 180

        self.green_min_area = 8000
        self.green_hsv_lower = np.array([40, 50, 60])
        self.green_hsv_upper = np.array([90, 255, 255])

        # Stuck detection
        self.stuck_sum_threshold = 5000
        self.stuck_boost = 30
        self.stuck_min_speed = 30
        self.error_sum = 0

        # Double green routine
        self.dg_spin = 75
        self.dg_spin_time = 1.5
        self.dg_reacquire_threshold = 1
        self.dg_reacquire_timeout = 5

        # State
        self.last_angle = self.angle = 90
        self.turn = 0

        self.prev_side = None
        self.green_signal = None
        self.prev_green_signal = None
        self.last_seen_green = 0.0

        self.image = None
        self.gray_image = None
        self.display_image = None

        self.black_mask = None
        self.black_contour = None

        self.green_mask = None
        self.green_contours = []

    # ======================================================================
    # MAIN LOOP
    # ======================================================================

    def follow(self, starting: bool = False) -> None:
        self.image = line_camera.capture_array()
        self.display_image = self.image.copy() if self.image is not None else None

        self.find_black()

        if self.black_contour is not None:
            self.find_green()
            self.green_check()
            self.calculate_angle(self.black_contour)
            if not starting:
                self._turn()
            else:
                self.robot_state.debug_text.clear()
        else:
            if not starting:
                oled.text("GAP", 38, 12, size=30, clear=True)
                self.gap_handling()
            else:
                self.robot_state.debug_text.clear()


        if self.display_image is not None and line_camera.X11:
            show(
                self.display_image,
                display=line_camera.X11,
                name="line",
                debug_lines=self.robot_state.debug_text,
            )

    # ======================================================================
    # TURN
    # ======================================================================

    def _turn(self) -> None:
        # Double green
        if self.green_signal == "DOUBLE":
            self.robot_state.debug_text.append("DOUBLE GREEN")
            oled.text("DG", 35, 12, size=30, clear=True)

            self._run_percent(self.dg_spin, -self.dg_spin, self.dg_spin_time)

            self.run_till_camera(
                self.dg_spin - 10,
                -self.dg_spin + 10,
                threshold=self.dg_reacquire_threshold,
                text=["DOUBLE GREEN"],
                timeout=self.dg_reacquire_timeout,
            )

            return

        # Display status
        if self.green_signal == "LEFT":
            self.robot_state.debug_text.append("GREEN LEFT")
            oled.text("L", 50, 12, size=30, clear=True)
        elif self.green_signal == "RIGHT":
            self.robot_state.debug_text.append("GREEN RIGHT")
            oled.text("R", 50, 12, size=30, clear=True)
        elif self.green_signal == "APPROACH":
            self.robot_state.debug_text.append("GREEN APPROACH")
            oled.text("G", 50, 12, size=30, clear=True)
        else:
            oled.text("LINE", 25, 12, size=30, clear=True)

        # Error (positive means line is to the right of center)
        raw_error = float(self.angle - 90)

        # Running average to reduce twitching
        self.error_history.append(raw_error)
        average_error = sum(self.error_history) / len(self.error_history)

        # Proportional turn
        self.turn = int(self.proportional_gain * average_error)

        # Speed ramp: faster when the error is closer to 0
        error_magnitude = abs(average_error)

        ramp = error_magnitude / float(self.speed_ramp_error)
        if ramp < 0.0:
            ramp = 0.0
        elif ramp > 1.0:
            ramp = 1.0

        # Smooth ramp (prevents sudden speed jumps)
        ramp = ramp * ramp * (3.0 - 2.0 * ramp)

        base_speed = self.straight_speed + (self.corner_speed - self.straight_speed) * ramp

        # Keep turning authority so one motor does not hit the clamp too early
        base_speed = min(base_speed, 100 - abs(self.turn))

        # Cap speed if approaching green
        if self.green_signal == "APPROACH":
            base_speed = min(base_speed, self.speed_approach)

        left_percent = base_speed + self.turn
        right_percent = base_speed - self.turn

        left_percent = max(-100, min(100, left_percent))
        right_percent = max(-100, min(100, right_percent))

        stuck, left_percent, right_percent = self.stuck_check(left_percent, right_percent, average_error)

        left_percent = max(-100, min(100, left_percent))
        right_percent = max(-100, min(100, right_percent))

        self.robot_state.debug_text.append(f"ANGLE: {self.angle}")
        self.robot_state.debug_text.append(f"ERROR: {average_error:.1f}")
        self.robot_state.debug_text.append(f"TURN: {self.turn}")
        self.robot_state.debug_text.append(f"BASE: {base_speed:.1f}")
        self.robot_state.debug_text.append(f"SUM: {self.error_sum}")
        if stuck:
            self.robot_state.debug_text.append("STUCK")

        self._run_percent(left_percent, right_percent)


    def _run_percent(self, left_percent: float, right_percent: float, duration: float | None = None) -> None:
        left = max(-1.0, min(1.0, left_percent / 100.0))
        right = max(-1.0, min(1.0, right_percent / 100.0))
        if duration is None:
            motors.run(left, right)
        else:
            motors.run(left, right, duration)

    # ======================================================================
    # HELPERS
    # ======================================================================

    def run_till_camera(
        self,
        left_percent: float,
        right_percent: float,
        threshold: int,
        text: list[str] | None = None,
        timeout: float = 3.0,
    ) -> None:
        if text is None:
            text = []

        self._run_percent(left_percent, right_percent)

        start_time = time.perf_counter()
        while True:
            if time.perf_counter() - start_time > timeout or listener.get_mode() == 0:
                break

            time.sleep(0.001)

            self.image = line_camera.capture_array()
            self.display_image = self.image.copy() if self.image is not None else None

            self.find_black()
            if self.black_contour is not None:
                self.calculate_angle(self.black_contour)
            else:
                self.angle = 90

            if self.display_image is not None and line_camera.X11:
                show(self.display_image, display=line_camera.X11, name="line", debug_lines=text)

            if self.black_contour is not None and (90 - threshold) < self.angle < (90 + threshold):
                break

        motors.run(0, 0)

    def stuck_check(self, left_percent: float, right_percent: float, smoothed_error: float) -> tuple[bool, float, float]:
        if abs(smoothed_error) < 10:
            self.error_sum = 0
            return False, left_percent, right_percent

        self.error_sum += int(smoothed_error)

        if abs(self.error_sum) > self.stuck_sum_threshold:
            oled.text("STUCK", 20, 12, size=30, clear=True)

            left_percent += self.stuck_boost
            right_percent += self.stuck_boost

            if left_percent < self.stuck_min_speed:
                left_percent = self.stuck_min_speed
            if right_percent < self.stuck_min_speed:
                right_percent = self.stuck_min_speed

            return True, left_percent, right_percent

        return False, left_percent, right_percent

    # ======================================================================
    # GAP
    # ======================================================================

    def gap_handling(self) -> None:
        self.robot_state.debug_text.append("GAP")

        self._run_percent(-50, -50, 0.2)

        for _ in range(3):
            if self._wait_for_black_contour(timeout=0.8):
                break

        self._run_percent(-60, -60, 0.30)
        self.align_to_contour_angle(threshold=3, timeout=10)

        self._run_percent(70, 70, 0.5)

        if not self._move_and_check_black(0.5):
            if not self._move_and_check_black(0.5):
                if not self._move_and_check_black(0.5):
                    self._run_percent(-40, -40, 1.0)

        motors.run(0, 0)


    def _wait_for_black_contour(self, timeout: float = 1.0) -> bool:
        start_time = time.perf_counter()

        while True:
            if listener.get_mode() == 0 or time.perf_counter() - start_time > timeout:
                return False

            time.sleep(0.001)

            self.image = line_camera.capture_array()
            self.display_image = self.image.copy() if self.image is not None else None

            self.find_black()

            if self.black_contour is not None:
                bottom_y = line_camera.HEIGHT - 5
                touches_bottom = any(p[0][1] >= bottom_y for p in self.black_contour)
                if touches_bottom and cv2.contourArea(self.black_contour) > self.min_black_area:
                    return True


    def align_to_contour_angle(self, threshold: int = 4, timeout: float = 2.0) -> None:
        start_time = time.perf_counter()

        while True:
            if time.perf_counter() - start_time > timeout or listener.get_mode() == 0:
                break

            time.sleep(0.001)

            self.image = line_camera.capture_array()
            self.display_image = self.image.copy() if self.image is not None else None

            self.find_black(extra_erode=True)

            if self.black_contour is None or self.black_mask is None:
                self._run_percent(-40, -40)
                continue

            top_point = self.calculate_top_contour(self.black_contour)

            band_h = 2
            y0 = max(0, line_camera.HEIGHT - band_h)
            bottom_band = self.black_mask[y0:line_camera.HEIGHT, :]
            ys, xs = np.where(bottom_band == 255)

            if len(xs) == 0:
                self._run_percent(-40, -40)
                continue

            bottom_point = (int(np.mean(xs)), int(np.mean(ys)) + y0)

            dx = top_point[0] - bottom_point[0]
            dy = top_point[1] - bottom_point[1]
            angle = int(np.degrees(np.arctan2(dy, dx)))
            if angle < 0:
                angle += 180

            angle_error = angle - 90

            if abs(angle_error) < threshold:
                motors.run(0, 0)
                break

            if angle_error > 0:
                self._run_percent(35, -35)
            else:
                self._run_percent(-35, 35)

        motors.run(0, 0)


    def _move_and_check_black(self, duration: float) -> bool:
        self._run_percent(70, 70, duration)
        self._run_percent(0, 0, 0.05)

        self.image = line_camera.capture_array()
        self.display_image = self.image.copy() if self.image is not None else None

        self.find_black()
        return self.black_contour is not None


    # ======================================================================
    # ANGLE
    # ======================================================================

    def calculate_angle(self, contour=None, validate: bool = False):
        if contour is None:
            return 90

        ref_point = self.calculate_top_contour(contour)

        if self.green_signal in ("LEFT", "RIGHT"):
            self.prev_side = self.green_signal

            extreme_point = (
                min(contour, key=lambda p: p[0][0]) if self.green_signal == "LEFT"
                else max(contour, key=lambda p: p[0][0])
            )

            x, y = int(extreme_point[0][0]), int(extreme_point[0][1])
            center_x = line_camera.WIDTH // 2
            shift_pixels = 60

            if self.green_signal == "LEFT":
                x = min(x + shift_pixels, center_x)
            else:
                x = max(x - shift_pixels, center_x)

            x = max(0, min(line_camera.WIDTH - 1, x))

            ref_point = (x, y)
            return self._finalize_angle(ref_point, validate)


        if ref_point is not None:
            w = line_camera.WIDTH
            h = line_camera.HEIGHT
            edge_w = max(1, int(w / 16))

            left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= edge_w]
            right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= (w - edge_w)]

            if ref_point[1] < int(h / 4):
                return self._finalize_angle(ref_point, validate)

            if self.green_signal != "APPROACH" and (left_edge_points or right_edge_points):
                y_left = int(np.mean([p[1] for p in left_edge_points])) if left_edge_points else None
                y_right = int(np.mean([p[1] for p in right_edge_points])) if right_edge_points else None

                if self.prev_side is None:
                    if y_left is None and y_right is not None:
                        self.prev_side = "RIGHT"
                    elif y_right is None and y_left is not None:
                        self.prev_side = "LEFT"
                    elif y_left is not None and y_right is not None:
                        self.prev_side = "LEFT" if y_left < y_right else "RIGHT"

                if y_left is not None and (y_right is None or self.prev_side == "LEFT"):
                    ref_point = (0, y_left)
                    self.prev_side = "LEFT"
                elif y_right is not None:
                    ref_point = (w - 1, y_right)
                    self.prev_side = "RIGHT"

        return self._finalize_angle(ref_point, validate)

    def _finalize_angle(self, ref_point, validate: bool):
        bottom_center = (line_camera.WIDTH // 2, line_camera.HEIGHT)
        dx = bottom_center[0] - ref_point[0]
        dy = bottom_center[1] - ref_point[1]

        angle = int(np.degrees(np.arctan2(dy, dx)))
        if angle < 0:
            angle += 180

        if validate:
            return angle

        self.angle = angle
        self.last_angle = angle

        if self.display_image is not None and line_camera.X11 and ref_point is not None:
            cv2.circle(self.display_image, ref_point, int(line_camera.HEIGHT / 20), (255, 255, 0), 2)

        return angle

    def calculate_top_contour(self, contour):
        if contour is None:
            return (line_camera.WIDTH // 2, line_camera.HEIGHT)

        h, w = self.gray_image.shape[:2]

        contour_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

        min_y = int(np.min(contour[:, 0, 1]))
        min_y = max(0, min_y)

        band_h = max(4, int(h * 0.06))
        y0 = min_y
        y1 = min(h, y0 + band_h)

        band = np.zeros((h, w), dtype=np.uint8)
        band[y0:y1, :] = 255

        masked = cv2.bitwise_and(contour_mask, band)
        ys, xs = np.where(masked == 255)
        if len(xs) == 0:
            return (w // 2, h)

        return (int(np.mean(xs)), int(np.mean(ys)))

    # ======================================================================
    # GREEN
    # ======================================================================

    def green_hold(self):
        if self.green_signal not in [None, "DOUBLE", "APPROACH"] and self.prev_green_signal != self.green_signal:
            self.last_seen_green = time.perf_counter()
        elif self.green_signal == "DOUBLE":
            self.last_seen_green = 0

        if (time.perf_counter() - self.last_seen_green) < 0.5 and self.prev_green_signal not in [None, "DOUBLE", "APPROACH"]:
            self.green_signal = self.prev_green_signal

    def validate_green_contour(self, contour):
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect).astype(np.int32)

        y_sorted = sorted(box, key=lambda p: p[1])
        top_left, top_right = y_sorted[:2]

        x_avg = int((top_left[0] + top_right[0]) / 2)
        y_avg = int((top_left[1] + top_right[1]) / 2)

        if self.black_mask is None:
            return None, None

        check_y = y_avg - max(1, int(line_camera.HEIGHT / 20))
        if 0 <= x_avg < line_camera.WIDTH and 0 <= check_y < line_camera.HEIGHT:
            if self.black_check((x_avg, check_y)):
                return box, y_avg

        return None, None

    def black_check(self, check_point):
        if self.black_mask is None:
            return False

        x, y = check_point
        s = max(2, int(line_camera.WIDTH / 48))

        y0 = max(0, y - s)
        y1 = min(line_camera.HEIGHT, y + s)
        x0 = max(0, x - s)
        x1 = min(line_camera.WIDTH, x + s)
        region = self.black_mask[y0:y1, x0:x1]

        if self.display_image is not None and line_camera.X11:
            cv2.circle(self.display_image, (x, y), 2 * s, (0, 255, 255), 2)

        if not np.any(region == 255) or not self.green_contours:
            return False

        for yy in range(y0, y1):
            for xx in range(x0, x1):
                if self.black_mask[yy, xx] != 255:
                    continue
                in_green = any(cv2.pointPolygonTest(c, (xx, yy), False) >= 0 for c in self.green_contours)
                if not in_green:
                    return True
        return False

    def green_check(self):
        self.green_signal = None

        valid_rects, y_avgs = [], []
        for contour in self.green_contours:
            rect, y_avg = self.validate_green_contour(contour)
            if rect is not None:
                valid_rects.append(rect)
                y_avgs.append(y_avg)

        if len(valid_rects) > 1:
            self.green_signal = "DOUBLE"
        elif len(valid_rects) == 1:
            rect = valid_rects[0]
            y_avg = y_avgs[0]

            if y_avg < int(line_camera.HEIGHT / 2):
                self.green_signal = "APPROACH"
            else:
                left_points = sorted(rect, key=lambda p: p[0])[:2]
                left_x = int(sum(p[0] for p in left_points) / 2)
                left_y = int(sum(p[1] for p in left_points) / 2)

                check = (left_x - int(line_camera.WIDTH / 16), left_y)
                if self.black_check(check):
                    self.green_signal = "RIGHT"
                else:
                    right_points = sorted(rect, key=lambda p: p[0])[-2:]
                    right_x = int(sum(p[0] for p in right_points) / 2)
                    right_y = int(sum(p[1] for p in right_points) / 2)

                    check = (right_x + int(line_camera.WIDTH / 16), right_y)
                    if self.black_check(check):
                        self.green_signal = "LEFT"

        self.green_hold()
        self.prev_green_signal = self.green_signal
        self.robot_state.debug_text.append(f"GREEN: {self.green_signal}")

    def find_green(self):
        self.green_mask = None
        self.green_contours = []
        if self.image is None:
            return

        hsv = cv2.cvtColor(self.image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.green_hsv_lower, self.green_hsv_upper)

        mask = cv2.medianBlur(mask, 5)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.green_contours = [c for c in contours if cv2.contourArea(c) > self.green_min_area]

        keep = np.zeros_like(mask)
        for c in self.green_contours:
            cv2.drawContours(keep, [c], -1, 255, thickness=cv2.FILLED)
        self.green_mask = keep

        if self.green_contours and self.display_image is not None and line_camera.X11:
            cv2.drawContours(self.display_image, self.green_contours, -1, (255, 0, 255), max(1, int(line_camera.HEIGHT / 100)))

    # ======================================================================
    # BLACK
    # ======================================================================

    def _poly_mask(self, poly, h, w):
        m = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(m, [np.int32(poly)], 255)
        return m

    def _pick_black_contour(self, contours):
        if not contours:
            return None
        if len(contours) == 1:
            return contours[0]

        h = line_camera.HEIGHT
        w = line_camera.WIDTH

        bottom_y = h - max(4, int(h * 0.05))
        x0 = int(w * 0.20)
        x1 = int(w * 0.80)

        bottom_hit = []
        for c in contours:
            if any((p[0][1] >= bottom_y) and (x0 <= p[0][0] <= x1) for p in c):
                bottom_hit.append(c)

        candidates = bottom_hit if bottom_hit else contours

        close = []
        for c in candidates:
            ang = self.calculate_angle(c, validate=True)
            if abs(ang - self.last_angle) < 90:
                close.append(c)
        candidates = close if close else candidates

        tall = []
        mid_y = h // 2
        for c in candidates:
            highest_point = max(c, key=lambda p: p[0][1])
            if highest_point[0][1] > mid_y:
                tall.append(c)
        candidates = tall if tall else candidates

        return max(candidates, key=cv2.contourArea)

    def find_black(self, extra_erode: bool = False):
        self.black_contour = None
        self.black_mask = None

        if self.image is None:
            return

        self.gray_image = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        _, mask = cv2.threshold(self.gray_image, self.base_black, 255, cv2.THRESH_BINARY_INV)

        h, w = line_camera.HEIGHT, line_camera.WIDTH

        bright = np.zeros((h, w), dtype=np.uint8)
        bright |= self._poly_mask(line_camera.bright_top, h, w)
        bright |= self._poly_mask(line_camera.bright_left, h, w)
        bright |= self._poly_mask(line_camera.bright_right, h, w)
        bright |= self._poly_mask(line_camera.bright_bottom, h, w)

        _, mask_bright = cv2.threshold(self.gray_image, self.bright_black, 255, cv2.THRESH_BINARY_INV)
        inv_bright = cv2.bitwise_not(bright)
        mask = cv2.bitwise_or(cv2.bitwise_and(mask, inv_bright), cv2.bitwise_and(mask_bright, bright))

        if self.green_mask is not None:
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(self.green_mask))

        mask = cv2.medianBlur(mask, 5)

        if extra_erode:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        else:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > self.min_black_area]
        contours = [c for c in contours if any(p[0][1] > int(line_camera.HEIGHT * 0.5) for p in c)]

        if not contours:
            return

        self.black_contour = self._pick_black_contour(contours)
        if self.black_contour is None:
            return

        contour_mask = np.zeros_like(self.gray_image, dtype=np.uint8)
        cv2.drawContours(contour_mask, [self.black_contour], -1, 255, thickness=cv2.FILLED)
        self.black_mask = contour_mask

        if self.display_image is not None and line_camera.X11:
            cv2.drawContours(self.display_image, [self.black_contour], -1, (255, 255, 0), max(1, int(line_camera.HEIGHT / 100)))