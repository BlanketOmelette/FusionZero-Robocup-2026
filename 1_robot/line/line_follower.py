from core.shared_imports import time, cv2, np
from core.utilities import show
from hardware.robot import *

class LineFollower:
    def __init__(self, robot_state):
        self.robot_state = robot_state

        # Follower
        self.speed = 25
        self.turn_multi = 2.2
        self.rate_multi = 0.0
        self.integral_multi = 0.0 #0.002
        self.min_black_area = 1500

        # Black thresholding
        self.base_black = 110

        # State
        self.last_angle = self.angle = 90
        self.turn = self.integral = self.prev_error = 0
        self.prev_time = time.perf_counter()
        self.image = self.gray_image = self.black_mask = self.black_contour = self.display_image = None

    def follow(self, starting=False) -> None:
        frame = line_camera.capture_array()
        self.image = frame
        self.display_image = self.image.copy()

        self.find_black()

        if self.black_contour is not None:
            self.calculate_angle(self.black_contour)
            if not starting:
                self.__turn()
            else:
                self.robot_state.debug_text.clear()
        else:
            if not starting:
                oled.text("GAP", 38, 12, size=30, clear=True)
                motors.run(0, 0)
            else:
                self.robot_state.debug_text.clear()

        if self.display_image is not None and line_camera.X11:
            show(self.display_image, display=line_camera.X11, name="line", debug_lines=self.robot_state.debug_text)

    def __turn(self):
        error = self.angle - 90

        now = time.perf_counter()
        time_change = now - self.prev_time
        if time_change <= 1e-4: time_change = 1e-4

        if abs(error) < 5:
            self.turn = 0
        else:
            self.turn = self.turn_multi * error
            self.turn += self.rate_multi * (error - self.prev_error) / time_change
            self.integral += error * time_change
            self.turn += self.integral * self.integral_multi

        self.prev_error = error
        self.prev_time = now
        if abs(error) < 10: self.integral = 0

        self.robot_state.debug_text.append(f"ANG: {self.angle}")
        self.robot_state.debug_text.append(f"TURN: {self.turn}")
        self.robot_state.debug_text.append(f"INT: {self.integral}")

        oled.text("LINE", 25, 12, size=30, clear=True)

        # Percent speeds
        v1 = self.speed + self.turn
        v2 = self.speed - self.turn

        # Clamp percent range
        v1 = max(-100, min(100, v1))
        v2 = max(-100, min(100, v2))

        # Convert to [-1, 1] for your new motors.run
        motors.run(v1 / 100.0, v2 / 100.0)

    def calculate_angle(self, contour=None, validate=False):
        self.angle = 90
        if contour is None:
            return 90

        ref_point = self.calculate_top_contour(contour)
        bottom_center = (line_camera.WIDTH // 2, line_camera.HEIGHT)

        dx = bottom_center[0] - ref_point[0]
        dy = bottom_center[1] - ref_point[1]

        angle_radians = np.arctan2(dy, dx)
        angle = int(np.degrees(angle_radians))
        if angle < 0:
            angle += 180

        self.angle = angle
        self.last_angle = angle

        if self.display_image is not None and line_camera.X11:
            cv2.circle(self.display_image, ref_point, int(line_camera.HEIGHT / 20), (255, 255, 0), 2)

        return angle

    def calculate_top_contour(self, contour):
        # Pick a stable point from an upper band of the contour mask
        h, w = self.gray_image.shape[:2]

        contour_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

        min_y = max(0, int(h * 0.25))
        max_y = min(h, min_y + int(h * 0.08))

        band = np.zeros((h, w), dtype=np.uint8)
        band[min_y:max_y, :] = 255
        masked = cv2.bitwise_and(contour_mask, band)

        ys, xs = np.where(masked == 255)
        if len(xs) == 0:
            return w // 2, h

        return int(np.mean(xs)), int(np.mean(ys))

    def find_black(self):
        self.black_contour = None
        self.black_mask = None

        self.gray_image = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        _, mask = cv2.threshold(self.gray_image, self.base_black, 255, cv2.THRESH_BINARY_INV)

        # clean noise
        mask = cv2.medianBlur(mask, 5)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > self.min_black_area]
        contours = [c for c in contours if any(p[0][1] > int(line_camera.HEIGHT * 0.5) for p in c)]

        if not contours:
            return

        self.black_contour = max(contours, key=cv2.contourArea)

        contour_mask = np.zeros_like(self.gray_image, dtype=np.uint8)
        cv2.drawContours(contour_mask, [self.black_contour], -1, 255, thickness=cv2.FILLED)
        self.black_mask = contour_mask

        if line_camera.X11:
            cv2.drawContours(self.display_image, [self.black_contour], -1, (255, 255, 0), max(1, int(line_camera.HEIGHT / 100)))
