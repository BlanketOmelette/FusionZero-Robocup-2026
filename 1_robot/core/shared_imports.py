import time
_START = time.perf_counter()

# =========================
# Standard library
# =========================
import os
import sys
import math
import random
import shutil
import socket
import getpass
import operator
import subprocess
import traceback
from pathlib import Path
from collections import deque
from typing import Optional, Deque
from queue import Queue, Empty
import threading
import threading
import multiprocessing as mp

# =========================
# Core third party
# =========================
import cv2
import numpy as np

# =========================
# Raspberry Pi / Camera
# =========================
from RPi import GPIO
from gpiozero import DigitalOutputDevice
from rpi_hardware_pwm import HardwarePWM
from picamera2 import Picamera2
from libcamera import Transform

# =========================
# I2C / Adafruit
# =========================
import board
import busio
import digitalio

import adafruit_vl53l0x
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

def import_time_s() -> float:
    return time.perf_counter() - _START

print(f"Imported shared_imports in {import_time_s():.3f} seconds")