import time
from gpiozero import DigitalOutputDevice, PWMOutputDevice

# Left side pins
LPWM = 12
LIN1 = 21
LIN2 = 26

# Right side pins
RPWM = 13
RIN1 = 20
RIN2 = 16

# If you have STBY wired to a GPIO, set it here, otherwise leave as None
STBY = None  # e.g. 25

def drive(pwm, in1, in2, speed, seconds=1.0):
    # speed in [-1, 1]
    if speed == 0:
        pwm.value = 0
        in1.off(); in2.off()
        time.sleep(seconds)
        return

    pwm.value = abs(speed)
    if speed > 0:
        in1.on(); in2.off()
    else:
        in1.off(); in2.on()
    time.sleep(seconds)
    pwm.value = 0
    in1.off(); in2.off()
    time.sleep(0.3)

def main():
    lpwm = PWMOutputDevice(LPWM, frequency=200, initial_value=0)
    lin1 = DigitalOutputDevice(LIN1, initial_value=False)
    lin2 = DigitalOutputDevice(LIN2, initial_value=False)

    rpwm = PWMOutputDevice(RPWM, frequency=200, initial_value=0)
    rin1 = DigitalOutputDevice(RIN1, initial_value=False)
    rin2 = DigitalOutputDevice(RIN2, initial_value=False)

    stby = None
    if STBY is not None:
        stby = DigitalOutputDevice(STBY, initial_value=True)

    try:
        print("Left forward")
        drive(lpwm, lin1, lin2, 0.6, 1.0)

        print("Left reverse")
        drive(lpwm, lin1, lin2, -0.6, 1.0)

        print("Right forward")
        drive(rpwm, rin1, rin2, 0.6, 1.0)

        print("Right reverse")
        drive(rpwm, rin1, rin2, -0.6, 1.0)

        print("Done")
    finally:
        if stby is not None:
            stby.off(); stby.close()
        for d in (lpwm, lin1, lin2, rpwm, rin1, rin2):
            d.close()

if __name__ == "__main__":
    main()
