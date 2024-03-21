import threading
from time import sleep
import RPi.GPIO as IO
from simple_pid import PID 

import rospy
from std_msgs.msg import Float32

class MotorController:
    def __init__(self):
        '''
        GPIO Pin Numbers Config
        '''
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
        self.maxPos = 450

        self.independentControl = False # default value: False

        self.targetPos = 0 # initial target position
        self.tolerance = 2

        self.degrees1 = 0
        
        # Setup Control Thread
        self.motorsOn = threading.Event()
        self.control_thread = threading.Thread(target=self.motorController)

    def setupGPIO(self):
        IO.setwarnings(False)
        IO.setmode(IO.BCM)

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
    ROS Interface Functions
    '''
    def init(self, cameraHeight, p, i, d):
        rospy.Subscriber("/height", Float32, self.setDistanceCallback)
        rospy.Subscriber("/degrees", Float32, self.setDegreesCallback)
        rospy.init_node("control_listener_node", anonymous=True)
        rospy.on_shutdown(self.turnOff)
        self.motor1pid = PID(p, i, d)
        self.motor1pid.output_limits = (-40, 100)
        self.turnOn()
        self.setCameraHeight(cameraHeight)
        rospy.spin()

    def setDistanceCallback(self, height):
        rospy.loginfo("Calling Callback!")
        self.setCameraHeight(height.data)

    def setDegreesCallback(self, degrees):
        self.degrees1 = int(degrees.data)

    '''
    Motor Controller Functions
    '''
    def turnOn(self):
        rospy.loginfo("Turning on...")
        try:
            self.setupGPIO()
            self.motorsOn.set()
            self.motor1.start(0)
            self.motor2.start(0)
            self.control_thread.start()
            rospy.loginfo("Turned On Successfully")
        except Exception as e:
            rospy.loginfo("Error: Unable to start motor controller" + str(e))
    
    def turnOff(self):
        print("Turning off")
        rospy.loginfo("Turning OFF motor controller")
        try:
            self.motorsOn.clear()
            self.motor1.stop()
            self.motor2.stop()
            self.control_thread.join()
            IO.cleanup()
            rospy.loginfo("Turned OFF successfully")
        except Exception as e:
            rospy.loginfo("Error: Unable to stop motor controller" + str(e))

    # Set the camera height target in the PID controller 
    def setCameraHeight(self, targetPos):
        if self.minPos <= targetPos <= self.maxPos:
            rospy.loginfo("Updating Camera Height:" + str(targetPos))
            self.targetPos = targetPos
            self.motor1pid.setpoint = targetPos
        else:
            rospy.loginfo("Invalid Height Input")

    def motorController(self):
        # While motors not in the correct position and the stop motors flag is not set
        while self.motorsOn.is_set() and not rospy.is_shutdown():
            self.moveMotorsConnected()

    '''
    Uses one PID controller and treats both motors as 1
    '''
    def moveMotorsConnected(self):
        # If within tolerance stop
        # with self.degreesLock:
        if abs(self.degrees1 - self.targetPos) < self.tolerance:
            rospy.loginfo("AT TARGET")
            self.motor1.ChangeDutyCycle(0)
            self.motor2.ChangeDutyCycle(0)
      # else move motors
        else:
            motorSpeed = self.motor1pid(self.degrees1)
            rospy.loginfo("Speed: " + str(round(motorSpeed, 2)) + " Distance: " + str(self.degrees1 - self.targetPos)) 
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
