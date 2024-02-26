import time
import RPi.GPIO as IO

class cameraMount:
    def __init__(self):
        self.YELLOW1 = 26
        self.BLUE1 = 19
        self.YELLOW2 = 21
        self.BLUE2 = 20

        self.M1_1 = 13
        self.M1_2 = 6
        self.M2_1 = 12
        self.M2_2 = 16

        self.degrees1 = 0
        self.degrees2 = 0

        self.targetPos1 = None
        self.targetPos2 = None

        IO.setwarnings(False)
        IO.setmode(IO.BCM)

        IO.setup(self.YELLOW1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE1, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.YELLOW2, IO.IN, pull_up_down=IO.PUD_DOWN)
        IO.setup(self.BLUE2, IO.IN, pull_up_down=IO.PUD_DOWN)

        IO.add_event_detect(self.YELLOW1, IO.RISING, callback=self.motor1Callback, bouncetime=30)
        IO.add_event_detect(self.YELLOW2, IO.RISING, callback=self.motor2Callback, bouncetime=30)

        IO.setup(self.M1_1, IO.OUT)
        IO.setup(self.M1_2, IO.OUT)
        IO.setup(self.M2_1, IO.OUT)
        IO.setup(self.M2_2, IO.OUT)

        self.motor1_1 = IO.PWM(self.M1_1, 100)
        self.motor1_2 = IO.PWM(self.M1_2, 100)

        self.motor2_1 = IO.PWM(self.M2_1, 100)
        self.motor2_2 = IO.PWM(self.M2_2, 100)

    def motor1Callback(self, channel):
        blue = IO.input(self.BLUE1)
        yellow = IO.input(self.YELLOW1)
        if blue == yellow:
            self.degrees1 += 1
        elif not (blue == yellow):
            self.degrees1 -= 1
        if self.targetPos1 and self.degrees1 == self.targetPos1:
            self.motor1_1.stop()
            self.motor1_2.stop()
            self.targetPos1 = None
        # print("M1:", self.degrees1)

    def motor2Callback(self, channel):
        blue = IO.input(self.BLUE2)
        yellow = IO.input(self.YELLOW2)
        if blue == yellow:
            self.degrees2 += 1
        elif not (blue == yellow):
            self.degrees2 -= 1
        if self.targetPos2 and self.degrees2 == self.targetPos2:
            self.motor2_1.stop()
            self.motor2_2.stop()
            self.targetPos2 = None
        # print("M2:", self.degrees2)


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
            self.targetPos1 = None
            self.targetPos2 = None
            self.motor1_1.stop() 
            self.motor1_2.stop()
            self.motor2_1.stop()
            self.motor2_2.stop()
            print("Degrees out of range or invalid")

c = cameraMount()
c.moveMotors(90)
c.moveMotors(-40)
