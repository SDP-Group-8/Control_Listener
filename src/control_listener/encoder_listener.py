import RPi.GPIO as IO
import rospy
from gpiozero import RotaryEncoder

from std_msgs.msg import Float32

class EncoderListener:
    def __init__(self):
        '''
        GPIO Pin Numbers Config
        '''
        self.YELLOW2 = 26
        self.BLUE2 = 19
        self.YELLOW1 = 21
        self.BLUE1 = 20

        '''
        Motor Controller Setup
        '''
        self.degrees1 = 0
        self.degrees2 = 0

        self.encoder = RotaryEncoder(a=self.YELLOW1, b=self.BLUE1, max_steps=10000)
        self.encoder.when_rotated = self.publisherCallback
        # self.encoder.max_steps = 10000
        # self.encoder.min_steps = -10000
        # self.encoder.when_rotated = self.publisherCallback

    def setupGPIO(self):
        IO.setwarnings(False)
        IO.setmode(IO.BCM)

        IO.setup(self.YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

        # IO.add_event_detect(self.YELLOW1, IO.RISING, callback=self.motor1Callback, bouncetime=2)

    '''
    Encoder Callbacks to Detect Motor Position
    '''
    
    # def motor1Callback(self, channel):
    #     # Yellow1 Pin = 21
    #     # Blue1 Pin = 20
    #     if (IO.input(21) == IO.input(20)):
    #         self.degrees1 += 1
    #     else:
    #         self.degrees1 -= 1

    #     self.degrees_publisher.publish(float(self.degrees1))

    def publisherCallback(self):
        self.degrees_publisher.publish(float(self.encoder.steps))
    '''
    ROS Interface Functions
    '''
    def init(self):
        self.degrees_publisher = rospy.Publisher("/degrees", Float32)
        self.setupGPIO()
        rospy.init_node("control_listener_node", anonymous=True)
        rospy.spin()
