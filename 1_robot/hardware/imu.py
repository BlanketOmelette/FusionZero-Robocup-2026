import math
from core.shared_imports import board, busio, time
from core.utilities import debug

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR


def _quat_to_euler_deg(q) -> tuple[float, float, float]:
    # Expect quaternion as (x, y, z, w) or (i, j, k, real)
    x, y, z, w = q[0], q[1], q[2], q[3]

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class IMU:
    def __init__(self, i2c=None, address: int = 0x4B) -> None:
        self.i2c = i2c if i2c is not None else busio.I2C(board.SCL, board.SDA)
        self.address = address

        self.imu = BNO08X_I2C(self.i2c, address=self.address)

        # enable reports
        try:
            self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        except Exception:
            pass
        try:
            self.imu.enable_feature(BNO_REPORT_ACCELEROMETER)
        except Exception:
            pass
        try:
            self.imu.enable_feature(BNO_REPORT_GYROSCOPE)
        except Exception:
            pass

        self._last_angles = None  # (roll, pitch, yaw)
        self._unwrap = None       # unwrapped yaw if you want later

        debug(["INITIALISATION", "IMU", "✓"], [25, 25, 50])

    def read(self) -> list[int] | None:
        # Try multiple quaternion property names (your new repo does this)
        quat = None
        for qname in ("quaternion", "game_quaternion", "geomagnetic_quaternion"):
            if hasattr(self.imu, qname):
                try:
                    quat = getattr(self.imu, qname)
                except Exception:
                    quat = None
                break

        if quat is None:
            return None

        roll, pitch, yaw = _quat_to_euler_deg(quat)
        self._last_angles = (roll, pitch, yaw)

        # Old repo style: return ints
        return [int(roll), int(pitch), int(yaw)]

    def close(self) -> None:
        # BNO08X lib does not always expose a close; safe no-op
        debug(["TERMINATION", f"IMU", "✓"], [25, 25, 50])
