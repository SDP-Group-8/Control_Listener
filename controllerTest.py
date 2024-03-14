from cameraMountController import cameraMount
import time

def main():
    global c
    c = cameraMount()
    c.turnOn()
    c.setCameraHeight(150)
    time.sleep(2)
    c.turnOff()
    c.setCameraHeight(30)
    time.sleep(2)

if __name__ ==  '__main__':
    try:
        main()
    except KeyboardInterrupt:
        c.motor1.stop(0)
        c.motor2.stop(0)