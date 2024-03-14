import time
import RPi.GPIO as IO
from simple_pid import PID 

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
        self.maxPos = 1000

        self.targetPos = 0
        self.tolerance = 2
        self.motor1pid = PID(10, 4, 0)
        self.motor2pid = PID(0, 0, 0)

        # PID bounds set so controller can half the speed of the motor.
        self.motor1pid.output_limits = (-100, 100)
        self.motor2pid.output_limits = (-100, 100)


        '''
        GPIO Setup
        '''
        IO.setwarnings(False)
        IO.setmode(IO.BCM)

        IO.setup(self.YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

        IO.add_event_detect(self.YELLOW1, IO.RISING, callback=self.motor1Callback, bouncetime=10)
        # IO.add_event_detect(self.YELLOW2, IO.RISING, callback=self.motor2Callback, bouncetime=10)

        IO.setup(self.ENA, IO.OUT)
        IO.setup(self.ENB, IO.OUT)
        IO.setup(self.M1_1, IO.OUT)
        IO.setup(self.M1_2, IO.OUT)
        IO.setup(self.M2_1, IO.OUT)
        IO.setup(self.M2_2, IO.OUT)

        self.motor1 = IO.PWM(self.ENA, 500)
        self.motor2 = IO.PWM(self.ENB, 500)

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

        # self.motor2pid.setpoint = self.degrees1
        
        # # Get PID output and set motor speed accordingly
        # motor1Speed = self.motor1pid(self.degrees1) 
        # print(motor1Speed)

        # # Set speed
        # self.setMotor1Speed(abs(motor1Speed))
        # self.setMotor2Speed(abs(motor1Speed))

        # # Set direction
        # if motor1Speed <= 0:
        #     self.setMotor1Direction("up")
        #     self.setMotor2Direction("up")
        # elif motor1Speed > 0:
        #     self.setMotor1Direction("down")
        #     self.setMotor2Direction("down")

        # if abs(self.targetPos - self.degrees1) < self.tolerance:
        #     # print("Tolerance detected")
        #     self.setMotor1Speed(0)
 
    # def motor2Callback(self, channel):
    #     # Read motor encoder inputs
    #     blue = IO.input(self.BLUE2)
    #     # Update motor position
    #     if blue:
    #         self.degrees2 += 1
    #     else:
    #         self.degrees2 -= 1

    #     if abs(self.targetPos - self.degrees2) < self.tolerance:
    #         # print("Tolerance met")
    #         self.setMotor2Speed(0)

    #     # Get PID output and set motor speed accordingly
    #     motor2Speed = self.motor2pid(self.degrees2) 

    #     # Set speed
    #     self.setMotor2Speed(abs(motor2Speed))

    #     # Set direction
    #     if motor2Speed <= 0:
    #         self.setMotor2Direction("up")
    #     elif motor2Speed > 0:
    #         self.setMotor2Direction("down")

    #     if abs(self.targetPos - self.degrees2) < self.tolerance:
    #         # print("Tolerance met")
    #         self.setMotor2Speed(0)

    # Set the camera height target in the PID controller 
    def setCameraHeight(self, targetPos):
        if self.minPos <= targetPos <= self.maxPos:
            self.targetPos = targetPos
            self.motor1pid.setpoint = targetPos
            self.motor2pid.setpoint = targetPos
        else:
            print("Invalid Position Input")
    
    def runMotors(self):
        while self.degrees1 != self.targetPos:
            time.sleep(0.1)
            # TODO extract this as a separate function
            # Initial Run of PID
            motor1Speed = self.motor1pid(self.degrees1) 
            print("motor1Speed", motor1Speed)
            # Set speed
            self.setMotor1Speed(abs(motor1Speed))
            # Set direction
            if motor1Speed <= 0:
                self.setMotor2Direction("up")
            elif motor1Speed > 0:
                self.setMotor2Direction("down")
            # Initial Run of PID
            motor2Speed = self.motor2pid(self.degrees2) 
            # Set speed
            self.setMotor2Speed(abs(motor2Speed))
            # Set direction
            if motor2Speed <= 0:
                self.setMotor2Direction("up")
            elif motor2Speed > 0:
                self.setMotor2Direction("down")


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
            print("Invalid Direction Input M1")

    def setMotor2Direction(self, direction):
        if direction == "up":
            IO.output(self.M2_1, IO.LOW)
            IO.output(self.M2_2, IO.HIGH)
        elif direction == "down":
            IO.output(self.M2_1, IO.HIGH)
            IO.output(self.M2_2, IO.LOW)
        else:
            print("Invalid Direction Input M2")
