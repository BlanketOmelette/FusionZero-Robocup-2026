from hardware.lasers import Lasers
from hardware.touch import Touch
from hardware.motors import Motors
from hardware.camera import Camera
from hardware.imu import IMU
from hardware.led import LED
from hardware.oled import OLED
from core.shared_imports import GPIO
GPIO.setmode(GPIO.BCM)


oled = OLED()

line_camera = Camera("line")
oled.text("LineCam ✓", 0, 0)

evac_camera = Camera("evac")
oled.text("EvacCam ✓", 0, 15)

motors = Motors()
oled.text("Motor ✓", 0, 30)

lasers = Lasers()
oled.text("ToF ✓", 0, 45)

imu = IMU()
oled.text("IMU ✓", 70, 0)

touch = Touch()
oled.text("Touch ✓", 70, 15)

led = LED()
oled.text("LED ✓", 70, 30)