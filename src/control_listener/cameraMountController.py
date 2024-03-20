import threading
import time
import RPi.GPIO as IO
from simple_pid import PID 

import rospy
from std_msgs.msg import Float32

class cameraMountController:
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
        self.maxPos = 500

        self.independentControl = False # default value: False

        self.targetPos = 0 # initial target position
        self.tolerance = 4
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

        self.motor1pid = PID(4, 2, 0.2)
        self.motor2pid = PID(0, 0, 0)

        # PID bounds set so controller can half the speed of the motor.
        self.motor1pid.output_limits = (-70, 100)
        self.motor2pid.output_limits = (-70, 100)

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
    ROS Interface Functions
    '''
    def init(self):
        rospy.Subscriber("/cmd_vel", Float32, self.setDistanceCallback)
        rospy.init_node("control_listener_node", anonymous=True)
        rospy.spin()
        self.turnOn()
        self.setCameraHeight(0)

    def setDistanceCallback(self, Float32):
        self.setCameraHeight(Float32.data)

    # TODO ~ Add ROS Subscriber for turning on/off motors?

    '''
    Motor Controller Functions
    '''
    def turnOn(self):
        print("Turning on")
        try:
            self.motorsOn.set()
            self.motor1.start(0)
            self.motor2.start(0)
            control_thread = threading.Thread(target=self.motorController)
            control_thread.start()
            return True
        except:
            print("Error: Unable to start motor controller")
            return False
    
    def turnOff(self):
        print("Turning off")
        try:
            self.motorsOn.clear()
            self.motor1.stop()
            self.motor2.stop()
            return True
        except:
            print("Error: Unable to stop motor controller")
            return False

    # Set the camera height target in the PID controller 
    def setCameraHeight(self, targetPos):
        if self.minPos <= targetPos <= self.maxPos:
            print("Updating Camera Height", targetPos)
            self.targetPos = targetPos
            self.motor1pid.setpoint = targetPos
            self.motor2pid.setpoint = targetPos
        else:
            print("Invalid Position Input")

    def motorController(self):
        # While motors not in the correct position and the stop motors flag is not set
        while self.motorsOn.is_set():
                if self.independentControl:
                    self.moveMotorsIndependently()
                else:
                    self.moveMotorsConnected()
                time.sleep(0.2)

    def moveMotorsIndependently(self):
        '''
        Motor 1
        '''
        if abs(self.degrees1 - self.targetPos) < self.tolerance:
            print("Within Tolerance")
            self.motor1.ChangeDutyCycle(0)
        else:
            # Calculate what speed should be
            motor1Speed = self.motor1pid(self.degrees1)
            # Set Speed
            self.motor1.ChangeDutyCycle(abs(motor1Speed))
            # Set direction
            if motor1Speed <= 0:
                IO.output(self.M1_1, IO.HIGH)
                IO.output(self.M1_2, IO.LOW)
            elif motor1Speed > 0:
                IO.output(self.M1_1, IO.LOW)
                IO.output(self.M1_2, IO.HIGH)

        '''
        Motor 2
        '''
        if abs(self.degrees2 - self.targetPos) < self.tolerance:
            self.motor2.ChangeDutyCycle(0)
        else:
            # Calculate what speed should be
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
    Uses one PID controller and treats both motors as 1
    '''
    def moveMotorsConnected(self):
        if abs(self.degrees1 - self.targetPos) < self.tolerance:
            print("Within Tolerance", abs(self.degrees1 - self.targetPos))
            self.motor1.ChangeDutyCycle(0)
            self.motor2.ChangeDutyCycle(0)
        else:
            # Get Value from PID
            motorSpeed = self.motor1pid(self.degrees1)
            print("Motor Speed:", motorSpeed, "Distance:", (self.degrees1 - self.targetPos))
            # print("Position:", self.degrees1)

            # Set speed
            self.motor1.ChangeDutyCycle(abs(motorSpeed))
            self.motor2.ChangeDutyCycle(abs(motorSpeed))
            # Set direction
            if motorSpeed <= 0:
                IO.output(self.M1_1, IO.HIGH)
                IO.output(self.M1_2, IO.LOW)
                IO.output(self.M2_1, IO.LOW)
                IO.output(self.M2_2, IO.HIGH)
            elif motorSpeed > 0:
                IO.output(self.M1_1, IO.LOW)
                IO.output(self.M1_2, IO.HIGH)
                IO.output(self.M2_1, IO.HIGH)
                IO.output(self.M2_2, IO.LOW)