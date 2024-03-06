import time
import RPi.GPIO as IO

class cameraMount:
    def __init__(self):
        '''
        GPIO Pin Numbers Config
        '''
        self.YELLOW1 = 26
        self.BLUE1 = 19
        self.YELLOW2 = 21
        self.BLUE2 = 20

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

        self.targetPos1 = None
        self.targetPos2 = None

        '''
        GPIO Setup
        '''
        IO.setwarnings(False)
        IO.setmode(IO.BCM)

        IO.setup(self.YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

        IO.add_event_detect(self.YELLOW1, IO.RISING, callback=self.motor1Callback, bouncetime=30)
        IO.add_event_detect(self.YELLOW2, IO.RISING, callback=self.motor2Callback, bouncetime=30)

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

    def motor1Callback(self):
        # Read motor encoder inputs
        blue = IO.input(self.BLUE1)
        yellow = IO.input(self.YELLOW1)
        # Update motor position
        if blue == yellow:
            self.degrees1 += 1
        elif not (blue == yellow):
            self.degrees1 -= 1
        '''
        Move motors accordingly to target position
        '''
        # If target position is reached, stop motors
        if self.degrees1 == self.targetPos1:
            self.motor1.setspeed(0)
            self.motor1.setspeed(0)
        # If target position too low move motors up
        elif self.degrees1 < self.targetPos1:
            self.setMotor1Direction("up")
            self.setMotor1Speed(100)
        # If target position too high move motors down
        elif self.degrees1 > self.targetPos1:
            self.setMotor2Direction("down")
            self.setMotor2Speed(100)

    def motor2Callback(self):
        # Read motor encoder inputs
        blue = IO.input(self.BLUE2)
        yellow = IO.input(self.YELLOW2)
        # Update motor position
        if blue == yellow:
            self.degrees2 += 1
        elif not (blue == yellow):
            self.degrees2 -= 1
        '''
        Move motors accordingly to target position
        '''
        # If target position is reached, stop motors
        if self.degrees2 == self.targetPos2:
            self.motor2.setspeed(0)
            self.motor2.setspeed(0)
        # If target position too low move motors up
        elif self.degrees2 < self.targetPos2:
            self.setMotor2Direction("up")
            self.setMotor2Speed(100)
        # If target position too high move motors down
        elif self.degrees2 > self.targetPos2:
            self.setMotor2Direction("down")
            self.setMotor2Speed(100)

    def moveMotors(self, degrees):
        targetPos = self.degrees1 + degrees
        if 0 <= targetPos <= 134:
            self.targetPos1 = targetPos
            self.targetPos2 = targetPos
            print("Target Position:", targetPos)
            if degrees > 0:
                print("Moving Forwards")
                self.motor1_1.start(100)
                self.motor1_2.start(0)
                self.motor2_1.start(100)
                self.motor2_2.start(0)
            elif degrees < 0:
                print("Moving Backwards")
                self.motor1_1.start(0)
                self.motor1_2.start(100)
                self.motor2_1.start(0)
                self.motor2_2.start(100)
            else:
                print("Invalid degrees")
            
            while self.targetPos1 and self.targetPos2:
                print("Moving...")
                time.sleep(0.5)
            print("Done Moving")
        else: 
            print("Degrees out of range or invalid")

    def setCameraHeight(self, position):
        if self.minPos <= position <= self.maxPos:
            self.targetPos1 = position
            self.setMotor1Speed(100)
            self.setMotor2Speed(100)
            self.setMotor1Direction("up")
            self.setMotor2Direction("up")

        else:
            print("Invalid Position Input")

    '''
    Utility Functions for setting motor direction
    '''
    def setMotor1Speed(self, speed):
        self.motor1.ChangeDutyCycle(speed)

    def setMotor2Speed(self, speed):
        self.motor2.ChangeDutyCycle(speed)

    def setMotor1Direction(self, direction):
        if direction == "up":
            IO.output(self.M1_1, IO.HIGH)
            IO.output(self.M1_2, IO.LOW)
        elif direction == "down":
            IO.output(self.M1_1, IO.LOW)
            IO.output(self.M1_2, IO.HIGH)
        else:
            print("Invalid Direction Input")

    def setMotor2Direction(self, direction):
        if direction == "up":
            IO.output(self.M2_1, IO.HIGH)
            IO.output(self.M2_2, IO.LOW)
        elif direction == "down":
            IO.output(self.M2_1, IO.LOW)
            IO.output(self.M2_2, IO.HIGH)
        else:
            print("Invalid Direction Input")