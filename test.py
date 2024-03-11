from motors import cameraMount
import time

def main():
    global c
    c = cameraMount()
    c.setCameraHeight(80)

    while True:
        print("position", c.degrees1, c.degrees2)
        time.sleep(0.5)

if __name__ ==  '__main__':
    try:
        main()
    except KeyboardInterrupt:
        c.motor1.ChangeDutyCycle(0)
        c.motor2.ChangeDutyCycle(0)