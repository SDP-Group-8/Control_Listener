import threading
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
        Config Variables
        '''
        self.minPos = 0
        self.maxPos = 200

        self.targetPos = 0 # initial target position
        self.tolerance = 2

        '''
        GPIO Setup
        '''
        IO.setwarnings(False)
        IO.setmode(IO.BCM)

        IO.setup(self.YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

        IO.add_event_detect(self.YELLOW1, IO.RISING, callback=self.motor1Callback)
        IO.add_event_detect(self.YELLOW2, IO.RISING, callback=self.motor2Callback)

        IO.setup(self.ENA, IO.OUT)
        IO.setup(self.ENB, IO.OUT)
        IO.setup(self.M1_1, IO.OUT)
        IO.setup(self.M1_2, IO.OUT)
        IO.setup(self.M2_1, IO.OUT)
        IO.setup(self.M2_2, IO.OUT)

        '''
        Create Motors
        '''
        self.motor1 = IO.PWM(self.ENA, 500)
        self.motor2 = IO.PWM(self.ENB, 500)

        '''
        Motor Controller Setup
        '''
        self.degrees1 = 0
        self.degrees2 = 0

        self.motor1pid = PID(10, 4, 0)
        self.motor2pid = PID(0, 0, 0)

        # PID bounds set so controller can half the speed of the motor.
        self.motor1pid.output_limits = (-100, 100)
        self.motor2pid.output_limits = (-100, 100)

        self.motorsOn = threading.Event()

    '''
    Encoder Callbacks to Detect Motor Position
    '''
    def motor1Callback(self, channel):
        # Read motor encoder inputs
        blue = IO.input(self.BLUE1)
        # Update motor position
        if blue:
            self.degrees1 += 1
        else:
            self.degrees1 -= 1

    def motor2Callback(self, channel):
        # Read motor encoder inputs
        blue = IO.input(self.BLUE2)
        # Update motor position
        if blue:
            self.degrees2 += 1
        else:
            self.degrees2 -= 1

    '''
    Motor Controller Functions
    '''
    def turnOn(self):
        try:
            self.motorsOn.set()
            control_thread = threading.Thread(target=self.motorController)
            control_thread.start()
            return True
        except:
            print("Error: Unable to start motor controller")
            return False
    
    def turnOff(self):
        try:
            self.motorsOn.clear()
            return True
        except:
            print("Error: Unable to stop motor controller")
            return False

    # Set the camera height target in the PID controller 
    def setCameraHeight(self, targetPos):
        if self.minPos <= targetPos <= self.maxPos:
            self.targetPos = targetPos
            self.motor1pid.setpoint = targetPos
            self.motor2pid.setpoint = targetPos
        else:
            print("Invalid Position Input")

    def motorController(self):
        # While motors not in the correct position and the stop motors flag is not set
        while self.motorsOn.is_set() and abs(self.degrees1 - self.targetPos) < self.tolerance and abs(self.degrees2 - self.targetPos) < self.tolerance:
                '''
                Motor 1 Control
                '''
                # Get Value from PID
                motor1Speed = self.motor1pid(self.degrees1)

                # Set speed
                self.motor1.ChangeDutyCycle(abs(motor1Speed))
                # Set direction
                if motor1Speed <= 0:
                    IO.output(self.M1_1, IO.HIGH)
                    IO.output(self.M1_2, IO.LOW)
                elif motor1Speed > 0:
                    IO.output(self.M1_1, IO.LOW)
                    IO.output(self.M1_2, IO.HIGH)
                
                '''
                Motor 2 Control
                '''
                # Initial Run of PID
                motor2Speed = self.motor2pid(self.degrees2) 
                # Set speed
                self.motor2.ChangeDutyCycle(abs(motor2Speed))
                # Set direction
                if motor2Speed <= 0:
                    IO.output(self.M2_1, IO.LOW)
                    IO.output(self.M2_2, IO.HIGH)
                elif motor2Speed > 0:
                    IO.output(self.M2_1, IO.HIGH)
                    IO.output(self.M2_2, IO.LOW)

                '''
                Control Loop Speed
                '''
                time.sleep(0.1)