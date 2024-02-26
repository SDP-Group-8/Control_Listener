import signal
import time
import sys
import RPi.GPIO as IO

class cameraMount:
    def __init__(self):
	

	self.YELLOW1 = 26
	self.BLUE1 = 19
	self.YELLOW2 = 21
	self.BLUE2 = 20

	self.M1_1 = 13
	self.M1_2 = 6
	self.M2_1 = 12
	self.M2_2 = 16

	self.degrees1 = 0
	degrees2 = 0

def signal_handler(sig, frame):
    IO.cleanup()
    sys.exit(0)

def motor1Callback(channel):
    global degrees1
    blue = IO.input(BLUE1)
    yellow = IO.input(YELLOW1)
    if blue == yellow:
        degrees1 += 1
    elif not (blue == yellow):
        degrees1 -= 1
    print("M1:", degrees1)

def motor2Callback(channel):
    global degrees2
    blue = IO.input(BLUE2)
    yellow = IO.input(YELLOW2)
    if blue == yellow:
        degrees2 += 1
    elif not (blue == yellow):
        degrees2 -= 1
    print("M2:", degrees2)

IO.setwarnings(False)
IO.setmode(IO.BCM)

IO.setup(YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
IO.setup(BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
IO.setup(YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
IO.setup(BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

IO.setup(M1_1, IO.OUT)
IO.setup(M1_2, IO.OUT)
IO.setup(M2_1, IO.OUT)
IO.setup(M2_2, IO.OUT)

motor1_1 = IO.PWM(M1_1, 100)
motor1_1.start(0)

motor1_2 = IO.PWM(M1_2, 100)
motor1_2.start(0)

motor2_1 = IO.PWM(M2_1, 100)
motor2_1.start(0)

motor2_2 = IO.PWM(M2_2, 100)
motor2_2.start(0)

IO.add_event_detect(YELLOW1, IO.RISING, callback=motor1Callback, bouncetime=300)
IO.add_event_detect(YELLOW2, IO.RISING, callback=motor2Callback, bouncetime=300)

while True:
    for x in range(50):
        motor1_1.ChangeDutyCycle(x)
	motor2_1.ChangeDutyCycle(x)
        time.sleep(0.1)
    for x in range(50):
        motor1_1.ChangeDutyCycle(50-x)
	motor2_1.ChangeDutyCycle(50-x)
        time.sleep(0.1)
