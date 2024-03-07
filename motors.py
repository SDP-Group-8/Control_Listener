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
        self.motor1_position = 0
        self.motor2_position = 0

        self.minPos = 0
        self.maxPos = 134
        self.motorSpeed = 100

        self.targetPos = 0

        self.tolerance = 0.1
        self.pid_controller = PID(0.5, 0.01, 0.05, setpoint=self.targetPos)
        # PID bounds set so controller can half the speed of the motor.
        self.pid_controller.output_limits = (self.motorSpeed // 2, self.motorSpeed) 

        '''
        GPIO Setup
        '''
        IO.setwarnings(False)
        IO.setmode(IO.BCM)

        IO.setup(self.YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

        IO.add_event_detect(self.YELLOW1, IO.RISING, callback=self.motor1Callback, bouncetime=5)
        IO.add_event_detect(self.YELLOW2, IO.RISING, callback=self.motor2Callback, bouncetime=5)

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
            self.motor1_position += 1
        else:
            self.motor1_position -= 1
        
        self.motor_speed = self.pid_controller(self.motor1_position)
        print(self.motor1_position, self.motor_speed)

        # Stop when motor is within a tolerance of the target position
        if abs(self.motor1_position - self.targetPos) <= self.tolerance:
            self.motor1.stop()

        elif self.motor2_position < self.targetPos:
            IO.output(self.M1_1, IO.HIGH)
            IO.output(self.M1_2, IO.LOW)
            self.motor1.start(self.motorSpeed)
        # If target position too high move motors down
        elif self.motor2_position > self.targetPos:
            IO.output(self.M1_1, IO.LOW)
            IO.output(self.M1_2, IO.HIGH)
            self.motor1.start(self.motorSpeed)


        


    def motor2Callback(self, channel):
        # Read motor encoder inputs
        blue = IO.input(self.BLUE2)
        # Update motor position
        if blue:
            self.motor2_position += 1
        else:
            self.motor2_position -= 1
        # print("Motor 2 Pos:", self.degrees2)
        '''
        Move motors accordingly to target position
        '''
        # If target position is reached, stop motors
        if self.targetPos == self.motor2_position:
            self.motor2.stop()
        # If target position too low move motors up
        elif self.motor2_position < self.targetPos:
            IO.output(self.M2_1, IO.HIGH)
            IO.output(self.M2_2, IO.LOW)
            self.motor2.start(self.motorSpeed)
        # If target position too high move motors down
        elif self.motor2_position > self.targetPos:
            IO.output(self.M2_1, IO.LOW)
            IO.output(self.M2_2, IO.HIGH)
            self.motor2.start(self.motorSpeed)

    def setCameraHeight(self, position):
        if self.minPos <= position <= self.maxPos:
            self.targetPos = position
            self.pid_controller.set_endpoint = position
            self.setMotor1Speed(self.motorSpeed)
            # self.setMotor2Speed(self.motorSpeed)
            self.setMotor1Direction("up")
            # self.setMotor2Direction("up")

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

if __name__ == '__main__':
    try:
        c = cameraMount()
        c.setCameraHeight(100)
        time.sleep(20)
        c.motor1.stop()
        c.motor2.stop()
    except KeyboardInterrupt:
        c.motor1.stop()
        c.motor2.stop()