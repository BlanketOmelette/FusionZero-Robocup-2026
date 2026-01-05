import time
from fusionzero.drivers.touch_sensors import TouchSensors, TouchPins

def main():
    t = TouchSensors(TouchPins(front_left=23, front_right=22, back_left=17, back_right=5))
    while True:
        raw_list = t.read_raw_list()
        raw_dict = t.read_raw_dict()
        pressed = t.read_pressed()
        print(f"raw_list [FL,FR,BL,BR]={raw_list}  raw_dict={raw_dict}  pressed={pressed}", end="\r")
        time.sleep(0.05)

if __name__ == "__main__":
    main()
