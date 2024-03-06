import RPi.GPIO as IO
import time

YELLOW2 = 26
BLUE2 = 19
YELLOW1 = 21
BLUE1 = 20

ENA = 13
M1_1 = 5
M1_2 = 6
ENB = 12
M2_1 = 7
M2_2 = 16

'''
Setup Variables
'''
degrees1 = 0
degrees2 = 0

minPos = 0
maxPos = 134

targetPos = 0

'''
GPIO Setup
'''
IO.setwarnings(False)
IO.setmode(IO.BCM)

IO.setup(YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
IO.setup(BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
IO.setup(YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
IO.setup(BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

IO.setup(ENA, IO.OUT)
IO.setup(ENB, IO.OUT)
IO.setup(M1_1, IO.OUT)
IO.setup(M1_2, IO.OUT)
IO.setup(M2_1, IO.OUT)
IO.setup(M2_2, IO.OUT)

motor1 = IO.PWM(ENA, 30)
motor2 = IO.PWM(ENB, 30)

'''
Motor Callbacks
'''
def motor1Callback(channel):
    global degrees1
    # Read motor encoder inputs
    blue = IO.input(BLUE1)
    yellow = IO.input(YELLOW1)
    # Update motor position
    if blue == yellow:
        degrees1 += 1
    elif not (blue == yellow):
        degrees1 -= 1

def motor2Callback(channel):
    global degrees2
    # Read motor encoder inputs
    blue = IO.input(BLUE2)
    yellow = IO.input(YELLOW2)
    # Update motor position
    if blue == yellow:
        degrees2 += 1
    elif not (blue == yellow):
        degrees2 -= 1

IO.add_event_detect(YELLOW1, IO.RISING, callback=motor1Callback, bouncetime=300)
IO.add_event_detect(YELLOW2, IO.RISING, callback=motor2Callback, bouncetime=300)

IO.output(M1_1, IO.LOW)
IO.output(M1_2, IO.HIGH)
motor1.start(100)
time.sleep(1)
motor1.stop()
time.sleep(1)
IO.output(M1_1, IO.HIGH)
IO.output(M1_2, IO.LOW)
motor1.start(100)
time.sleep(1)
motor1.stop()