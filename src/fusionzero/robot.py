from dataclasses import dataclass

import board
import busio

from fusionzero.drivers.tof_vl53l0x_multi import MultiVL53L0X
from fusionzero.drivers.bno08x_imu import BNO08xIMU
from fusionzero.drivers.touch_sensors import TouchSensors, TouchPins
from fusionzero.drivers.oled_ssd1306 import OledDisplay, OledConfig
from fusionzero.drivers.camera_csi import CsiCamera

@dataclass
class RobotConfig:
    # ToF
    tof_left_xshut: int = 10
    tof_right_xshut: int = 6

    # Cameras
    use_cameras: bool = False
    cam_width: int = 640
    cam_height: int = 480

    # Oled
    use_oled: bool = True
    oled_addr: int = 0x3C

class Robot:
    def __init__(self, cfg: RobotConfig = RobotConfig()):
        self.cfg = cfg

        # I2C bus for ToF/IMU/OLED etc
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Cameras
        self.cam0 = CsiCamera(0, width=cfg.cam_width, height=cfg.cam_height)
        self.cam1 = CsiCamera(1, width=cfg.cam_width, height=cfg.cam_height)

        # 3x ToF (forward no XSHUT, left/right with XSHUT)
        self.tof = MultiVL53L0X(
            self.i2c,
            left_xshut_gpio=cfg.tof_left_xshut,
            right_xshut_gpio=cfg.tof_right_xshut,
        )

        self.oled = None
        if cfg.use_oled:
            self.oled = OledDisplay(self.i2c, OledConfig(address=cfg.oled_addr))


        self.imu = BNO08xIMU(self.i2c, address=0x4B)
        self.touch = TouchSensors(
            TouchPins(front_left=23, front_right=22, back_left=17, back_right=5)
        )

    def read_sensors(self):
        imu_r = self.imu.read()
        data = {
            "tof": self.tof.read_dict(),
            "imu": {"yaw": imu_r.yaw_deg, "pitch": imu_r.pitch_deg, "roll": imu_r.roll_deg},
            "touch_raw": self.touch.read_raw_list(),
            "touch": self.touch.read_pressed(),
        }

        # Optional OLED status
        if self.oled is not None:
            tof = data["tof"]
            imu = data["imu"]
            touch = data["touch_raw"]

            def fmt(a):
                return "---" if a is None else f"{a:5.1f}"

            self.oled.show_lines([
                f"F {tof['forward']:4d} L {tof['left']:4d}",
                f"R {tof['right']:4d} T {touch}",
                f"Y {fmt(imu['yaw'])}",
                f"P {fmt(imu['pitch'])} R {fmt(imu['roll'])}",
            ])

        return data

    def close(self):
        for name in ("cam0", "cam1"):
            cam = getattr(self, name, None)
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    pass
