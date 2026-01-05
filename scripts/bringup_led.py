import time
from fusionzero.drivers.led_gpio import LedGPIO


def main():
    led = LedGPIO(pin_bcm=4, active_high=True, initial_on=False)
    print("LED bringup on BCM 4")

    try:
        led.on()
        time.sleep(1)
        led.off()
        time.sleep(1)

        led.blink(delay_s=0.5, times=2)

        print("Done")
    finally:
        led.close()


if __name__ == "__main__":
    main()
