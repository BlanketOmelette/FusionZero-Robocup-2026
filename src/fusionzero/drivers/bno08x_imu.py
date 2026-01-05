import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

Quat = Tuple[float, float, float, float]  # (x, y, z, w)


@dataclass
class ImuReading:
    quat: Optional[Quat]
    yaw_deg: Optional[float]
    pitch_deg: Optional[float]
    roll_deg: Optional[float]
    accel: Optional[Tuple[float, float, float]]
    gyro: Optional[Tuple[float, float, float]]
    mag: Optional[Tuple[float, float, float]]


def _quat_to_euler_deg(q: Quat) -> Tuple[float, float, float]:
    # q is (x, y, z, w)
    x, y, z, w = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class BNO08xIMU:
    def __init__(self, i2c, address: int = 0x4B, settle_s: float = 0.2):
        self.address = address
        self.imu = BNO08X_I2C(i2c, address=address)

        # Enable what we use. If a report is unsupported it may throw, so keep it soft.
        self.imu.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        try:
            self.imu.enable_feature(BNO_REPORT_GYROSCOPE)
        except Exception:
            pass

        try:
            self.imu.enable_feature(BNO_REPORT_MAGNETOMETER)
        except Exception:
            pass

        time.sleep(settle_s)

    def _read_quat(self) -> Optional[Quat]:
        # Most versions expose .quaternion; keep fallbacks.
        for name in ("quaternion", "game_quaternion", "geomagnetic_quaternion"):
            if hasattr(self.imu, name):
                try:
                    q = getattr(self.imu, name)
                    if q and len(q) == 4:
                        # BNO08X returns (i, j, k, real) which we treat as (x, y, z, w)
                        return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
                except Exception:
                    return None
        return None

    def read(self) -> ImuReading:
        accel = None
        gyro = None
        mag = None
        quat = None
        yaw = pitch = roll = None

        try:
            accel = self.imu.acceleration
        except Exception:
            accel = None

        # gyro naming differs by version
        for gname in ("gyroscope", "gyro"):
            if hasattr(self.imu, gname):
                try:
                    gyro = getattr(self.imu, gname)
                except Exception:
                    gyro = None
                break

        for mname in ("magnetic", "magnetometer"):
            if hasattr(self.imu, mname):
                try:
                    mag = getattr(self.imu, mname)
                except Exception:
                    mag = None
                break

        quat = self._read_quat()
        if quat is not None:
            roll, pitch, yaw = _quat_to_euler_deg(quat)

        return ImuReading(
            quat=quat,
            yaw_deg=yaw,
            pitch_deg=pitch,
            roll_deg=roll,
            accel=accel,
            gyro=gyro,
            mag=mag,
        )
