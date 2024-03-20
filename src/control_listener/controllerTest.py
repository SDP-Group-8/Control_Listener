from cameraMountController import cameraMountController
import time

def main():
    global c
    c = cameraMountController()
    c.turnOn()
    # Max height 300
    c.setCameraHeight(280)
    time.sleep(20)
    c.turnOff()

if __name__ ==  '__main__':
    try:
        main()
    except KeyboardInterrupt:
        c.turnOff()
