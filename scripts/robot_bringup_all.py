import time
from fusionzero.robot import Robot

def main():
    bot = Robot()
    while True:
        s = bot.read_sensors()
        print(s)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
