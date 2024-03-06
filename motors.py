import time
import RPi.GPIO as IO

class cameraMount:
    def __init__(self):
        '''
        GPIO Pin Numbers Config
        '''
        self.YELLOW2 = 26
        self.BLUE2 = 19
        self.YELLOW1 = 21
        self.BLUE1 = 20

        self.ENA = 13
        self.M1_1 = 5
        self.M1_2 = 6
        self.ENB = 12
        self.M2_1 = 7
        self.M2_2 = 16

        '''
        Setup Variables
        '''
        self.degrees1 = 0
        self.degrees2 = 0

        self.minPos = 0
        self.maxPos = 134
        self.motorSpeed = 100

        self.targetPos = 0

        '''
        GPIO Setup
        '''
        IO.setwarnings(False)
        IO.setmode(IO.BCM)

        IO.setup(self.YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

        IO.add_event_detect(self.YELLOW1, IO.RISING, callback=self.motor1Callback, bouncetime=300)
        IO.add_event_detect(self.YELLOW2, IO.RISING, callback=self.motor2Callback, bouncetime=300)

        IO.setup(self.ENA, IO.OUT)
        IO.setup(self.ENB, IO.OUT)
        IO.setup(self.M1_1, IO.OUT)
        IO.setup(self.M1_2, IO.OUT)
        IO.setup(self.M2_1, IO.OUT)
        IO.setup(self.M2_2, IO.OUT)

        self.motor1 = IO.PWM(self.ENA, 100)
        self.motor2 = IO.PWM(self.ENB, 100)

        self.motor1.start(0)
        self.motor2.start(0)

    def motor1Callback(self, channel):
        # Read motor encoder inputs
        blue = IO.input(self.BLUE1)
        # Update motor position
        if blue:
            self.degrees1 += 1
        else:
            self.degrees1 -= 1
        # print("Motor 1 Pos:", self.degrees1)
        '''
        Move motors accordingly to target position
        '''
        # If target position is reached, stop motors
        if self.degrees1 == self.targetPos:
            self.motor1.stop()
        # If target position too low move motors up
        elif self.degrees1 < self.targetPos:
            IO.output(self.M1_1, IO.LOW)
            IO.output(self.M1_2, IO.HIGH)
            self.motor1.start(self.motorSpeed)
        # If target position too high move motors down
        elif self.degrees1 > self.targetPos:
            IO.output(self.M1_1, IO.HIGH)
            IO.output(self.M1_2, IO.LOW)
            self.motor1.start(self.motorSpeed)

    def motor2Callback(self, channel):
        # Read motor encoder inputs
        blue = IO.input(self.BLUE2)
        # Update motor position
        if blue:
            self.degrees2 += 1
        else:
            self.degrees2 -= 1
        # print("Motor 2 Pos:", self.degrees2)
        '''
        Move motors accordingly to target position
        '''
        # If target position is reached, stop motors
        if self.degrees2 == self.targetPos:
            self.motor2.stop()
        # If target position too low move motors up
        elif self.degrees2 < self.targetPos:
            IO.output(self.M2_1, IO.HIGH)
            IO.output(self.M2_2, IO.LOW)
            self.motor2.start(self.motorSpeed)
        # If target position too high move motors down
        elif self.degrees2 > self.targetPos:
            IO.output(self.M2_1, IO.LOW)
            IO.output(self.M2_2, IO.HIGH)
            self.motor2.start(self.motorSpeed)

    def setCameraHeight(self, position):
        if self.minPos <= position <= self.maxPos:
            self.targetPos = position
            self.setMotor1Speed(self.motorSpeed)
            self.setMotor2Speed(self.motorSpeed)
            self.setMotor1Direction("up")
            self.setMotor2Direction("up")

        else:
            print("Invalid Position Input")

    '''
    Utility Functions for setting motor direction
    '''
    def setMotor1Speed(self, speed):
        print("Motor speed called")
        self.motor1.ChangeDutyCycle(speed)

    def setMotor2Speed(self, speed):
        print("Motor speed called")
        self.motor2.ChangeDutyCycle(speed)

    def setMotor1Direction(self, direction):
        print("Direction Changed")
        if direction == "up":
            IO.output(self.M1_1, IO.LOW)
            IO.output(self.M1_2, IO.HIGH)
        elif direction == "down":
            IO.output(self.M1_1, IO.HIGH)
            IO.output(self.M1_2, IO.LOW)
        else:
            print("Invalid Direction Input")

    def setMotor2Direction(self, direction):
        print("Direction Changed")
        if direction == "up":
            IO.output(self.M2_1, IO.HIGH)
            IO.output(self.M2_2, IO.LOW)
        elif direction == "down":
            IO.output(self.M2_1, IO.LOW)
            IO.output(self.M2_2, IO.HIGH)
        else:
            print("Invalid Direction Input")