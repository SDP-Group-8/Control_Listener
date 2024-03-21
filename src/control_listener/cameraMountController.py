import threading
from time import sleep
from time import perf_counter_ns
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
        self.tolerance = 2

        self.degreesLock = threading.Lock()

        '''
        Motor Controller Setup
        '''
        with self.degreesLock:
            self.degrees1 = 0
            self.degrees2 = 0

        self.motorSpeed = None
        # self.motor1pid = PID(4, 2, 0.1)
        # self.motor1pid = None
        # self.motor2pid = PID(0, 0, 0)

        # PID bounds set so controller can half the speed of the motor.
        

        # Setup Control Thread
        self.motorsOn = threading.Event()
        self.control_thread = threading.Thread(target=self.motorController)
        self.print_thread = threading.Thread(target=self.printLoop)
    def setupGPIO(self):
        IO.setwarnings(False)
        IO.setmode(IO.BCM)

        IO.setup(self.YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

        IO.add_event_detect(self.YELLOW1, IO.RISING, callback=self.motor1Callback, bouncetime=2)
        # IO.add_event_detect(self.YELLOW2, IO.RISING, callback=self.motor2Callback)

        IO.setup(self.ENA, IO.OUT)
        IO.setup(self.ENB, IO.OUT)
        IO.setup(self.M1_1, IO.OUT)
        IO.setup(self.M1_2, IO.OUT)
        IO.setup(self.M2_1, IO.OUT)
        IO.setup(self.M2_2, IO.OUT)

        # Create Motors
        self.motor1 = IO.PWM(self.ENA, 100)
        self.motor2 = IO.PWM(self.ENB, 100)

    '''
    Encoder Callbacks to Detect Motor Position
    '''
    
    def motor1Callback(self, channel):
        # Yellow1 Pin = 21
        # Blue1 Pin = 20
        if (IO.input(21) == IO.input(20)):
        # this will be clockwise rotation
            # with self.degreesLock:
            self.degrees1 += 1
        else:
            #with self.degreesLock:
            self.degrees1 -= 1
            
    # def motor2Callback(self, channel):
        # Yellow2 Pin = 26
        # Blue2 Pin = 19
      #  if (IO.input(26) == IO.input(19)):
       #     self.degrees2 += 1
       # else:
        #    self.degrees2 -= 1

    '''
    ROS Interface Functions
    '''
    def init(self, cameraHeight, p, i, d):
        rospy.Subscriber("/height", Float32, self.setDistanceCallback)
        rospy.init_node("control_listener_node", anonymous=True)
        rospy.on_shutdown(self.turnOff)
        # TODO move this??
        self.motor1pid = PID(p, i, d)
        # self.motor2pid = PID(p, i, d)
        self.motor1pid.output_limits = (-40, 100)
        # self.motor2pid.output_limits = (-50, 100)
        self.turnOn()
        self.setCameraHeight(cameraHeight)
        rospy.spin()
        self.turnOff()

    def setDistanceCallback(self, height):
        rospy.loginfo("Calling Callback!")
        self.setCameraHeight(height.data)

    '''
    Motor Controller Functions
    '''
    def turnOn(self):
        rospy.loginfo("Turning on...")
        try:
            self.setupGPIO()
            self.motorsOn.set()
            with self.degreesLock: 
                self.motor1.start(0)
                self.motor2.start(0)
            # self.setCameraHeight(0)
            self.control_thread.start()
            self.print_thread.start()
            rospy.loginfo("Turned On Successfully")
        except Exception as e:
            rospy.loginfo("Error: Unable to start motor controller" + str(e))
    
    def turnOff(self):
        print("Turning off")
        rospy.loginfo("Turning OFF motor controller")
        try:
            with self.degreesLock:
                self.motorsOn.clear()
                self.motor1.stop()
                self.motor2.stop()
            self.control_thread.join()
            self.print_thread.join()
            IO.cleanup()
            rospy.loginfo("Turned OFF successfully")
        except Exception as e:
            rospy.loginfo("Error: Unable to stop motor controller" + str(e))

    # Set the camera height target in the PID controller 
    def setCameraHeight(self, targetPos):
        if self.minPos <= targetPos <= self.maxPos:
            rospy.loginfo("Updating Camera Height:" + str(targetPos))
            with self.degreesLock:
                self.targetPos = targetPos
                self.motor1pid.setpoint = targetPos
                # self.motor2pid.setpoint = targetPos
        else:
            rospy.loginfo("Invalid Height Input")

    def motorController(self):
        # While motors not in the correct position and the stop motors flag is not set
        while self.motorsOn.is_set() and not rospy.is_shutdown():
                # rospy.loginfo("1:" + str(self.degrees1) + " 2:" + str(self.degrees2))
                sleep(0.5)
                # self.moveMotorsConnected()
                # Takes an average of 12.5ms to run. 50Hz = 20ms per run. so sleep 7.5ms
                # sleep(0.020)

    def printLoop(self):
        while self.motorsOn.is_set() and not rospy.is_shutdown():
            rospy.loginfo("degrees: " + str(self.degrees1))
            sleep(1)

    def moveMotorsIndependently(self):
        '''
        Motor 1
        '''
        if abs(self.degrees1 - self.targetPos) < self.tolerance:
            print("Within Tolerance")
            with self.degreesLock:
                self.motor1.ChangeDutyCycle(0)
        else:
            # Calculate what speed should be
            with self.degreesLock:
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
        with self.degreesLock:
#            motor2Speed = self.motor2pid(self.degrees2) 
        # Set speed
            motor2speed = 0
        if abs(self.degrees2 - self.targetPos) < self.tolerance:
            self.motor2.ChangeDutyCycle(0)
        else:
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
        # If within tolerance stop
        # with self.degreesLock:
        if abs(self.degrees1 - self.targetPos) < self.tolerance:
            #rospy.loginfo("AT TARGET")
            self.motor1.ChangeDutyCycle(0)
            self.motor2.ChangeDutyCycle(0)
      # else move motors
        else:
            self.motorSpeed = self.motor1pid(self.degrees1)
            #rospy.loginfo("Speed: " + str(round(motorSpeed, 2)) + " Distance: " + str(error)) 
            self.motor1.ChangeDutyCycle(abs(self.motorSpeed))
            self.motor2.ChangeDutyCycle(abs(self.motorSpeed))
            # Set direction
            if self.motorSpeed <= 0:
                IO.output(self.M1_1, IO.HIGH)
                IO.output(self.M1_2, IO.LOW)
                IO.output(self.M2_1, IO.LOW)
                IO.output(self.M2_2, IO.HIGH)
            elif self.motorSpeed > 0:
                IO.output(self.M1_1, IO.LOW)
                IO.output(self.M1_2, IO.HIGH)
                IO.output(self.M2_1, IO.HIGH)
                IO.output(self.M2_2, IO.LOW)
