import evdev
import rospy
import threading
from std_msgs.msg import Float32

class EncoderListener:
    def __init__(self):
        '''
        Motor Controller Setup
        '''
        self.steps = 0
        self.pin_a = 21
        self.pin_b = 20
        self.device = evdev.InputDevice('/dev/input/by-path/platform-rotary@' + str(hex(self.pin_a)[2:]) + '-event')
        self.read_encoder_thread = threading.Thread(target=self.read_encoder)

        self.stop = threading.Event()
        self.stop.clear()

    def read_encoder(self):
        for event in self.device.read_loop():
            if self.stop.is_set():
                self.device.close()
                break
            elif event.type == evdev.ecodes.EV_REL:
                self.steps -= event.value
                self.degrees_publisher.publish(float(self.steps))

    def shutdown(self):
        self.stop.set()
        self.read_encoder_thread.join()
    '''
    ROS Interface Functions
    '''
    def init(self):
        self.degrees_publisher = rospy.Publisher("/degrees", Float32)
        rospy.init_node("control_listener_node", anonymous=True)
        self.read_encoder_thread.start()
        rospy.on_shutdown(self.shutdown)
        rospy.spin()
