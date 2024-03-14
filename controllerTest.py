from cameraMountController import cameraMountController
import time

def main():
    global c
    c = cameraMountController()
    c.turnOn()
    c.setCameraHeight(90)
    time.sleep(60)
    c.turnOff()

if __name__ ==  '__main__':
    try:
        main()
    except KeyboardInterrupt:
        c.turnOff()