from cameraMountController import cameraMountController
import time

def main():
    global c
    c = cameraMountController()
    c.turnOn()
    # Max height 300
    c.setCameraHeight(280)
    while True:
        print(c.degrees1, c.degrees2)
        time.sleep(1)

if __name__ ==  '__main__':
    try:
        main()
    except KeyboardInterrupt:
        c.turnOff()