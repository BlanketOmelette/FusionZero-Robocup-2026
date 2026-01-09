import os, sys, time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from hardware.servos import Servos
servo = Servos()

for i in range(10):
    servo.grab.move(70, sleep_s=1) # Grab down
    time.sleep(2)
    servo.grab.move(160, sleep_s=1) # Grab Up
    time.sleep(2)
    servo.grab.move(230, sleep_s=1) # Grab Dump
    time.sleep(2)
    servo.dump.move(47, sleep_s=0.3) # Dump Right
    time.sleep(2)
    servo.dump.move(80, sleep_s=0.3) # Dump Center
    time.sleep(2)
    servo.dump.move(110, sleep_s=0.3) # Dump Left
    time.sleep(2)
time.sleep(2)