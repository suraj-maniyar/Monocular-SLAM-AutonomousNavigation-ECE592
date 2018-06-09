import RPi.GPIO as GPIO
import time
import socket
import sys
import tty
import termios

USE_CLIENT_SERVER = 0   # Decision to use client server or not

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class Motors:

    def __init__(self):

        # THESE ARE BCM PINS

        # Black wire of motor to the right!

        self.MOTOR_1 = 21  # RIGHT Up/Down Motor (Normal: CW)
        self.MOTOR_1_IN_1 = 16  # In 1
        self.MOTOR_1_IN_2 = 20  # In 2
        self.MOTOR_1_IN_1_STATE = 1
        self.MOTOR_1_IN_2_STATE = 0
        self.MOTOR_1_ON = 0

        self.MOTOR_2 = 7  # LEFT Up/Down Motor (Normal: CCW)
        self.MOTOR_2_IN_1 = 12  # In 1
        self.MOTOR_2_IN_2 = 1  # In 2
        self.MOTOR_2_IN_1_STATE = 0
        self.MOTOR_2_IN_2_STATE = 1
        self.MOTOR_2_ON = 0

        self.MOTOR_3 = 0  # RIGHT Forward/Back Motor (Normal: CW)
        self.MOTOR_3_IN_1 = 6  # In 1
        self.MOTOR_3_IN_2 = 5  # In 2
        self.MOTOR_3_IN_1_STATE = 1
        self.MOTOR_3_IN_2_STATE = 0
        self.MOTOR_3_ON = 0

        self.MOTOR_4 = 26  # LEFT Forward/Back Motor (Normal: CCW)
        self.MOTOR_4_IN_1 = 13  # In 1
        self.MOTOR_4_IN_2 = 19  # In 2
        self.MOTOR_4_IN_1_STATE = 0
        self.MOTOR_4_IN_2_STATE = 1
        self.MOTOR_4_ON = 0

        # Directions
        # 0 - None
        # 1 - Up
        # 2 - Down
        # 3 - Left
        # 4 - Right
        # 5 - Forward
        # 6 - Backward
        self.DIRECTION = 0

        self.WAIT_TIME = 0.25
        self.POWER = 30

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Setting up motor 1
        GPIO.setup(self.MOTOR_1, GPIO.OUT)
        GPIO.setup(self.MOTOR_1_IN_1, GPIO.OUT)
        GPIO.setup(self.MOTOR_1_IN_2, GPIO.OUT)
        GPIO.output(self.MOTOR_1_IN_1, self.MOTOR_1_IN_1_STATE)
        GPIO.output(self.MOTOR_1_IN_2, self.MOTOR_1_IN_2_STATE)

        # Setting up motor 2
        GPIO.setup(self.MOTOR_2, GPIO.OUT)
        GPIO.setup(self.MOTOR_2_IN_1, GPIO.OUT)
        GPIO.setup(self.MOTOR_2_IN_2, GPIO.OUT)
        GPIO.output(self.MOTOR_2_IN_1, self.MOTOR_2_IN_1_STATE)
        GPIO.output(self.MOTOR_2_IN_2, self.MOTOR_2_IN_2_STATE)

        # Setting up motor 3
        GPIO.setup(self.MOTOR_3, GPIO.OUT)
        GPIO.setup(self.MOTOR_3_IN_1, GPIO.OUT)
        GPIO.setup(self.MOTOR_3_IN_2, GPIO.OUT)
        GPIO.output(self.MOTOR_3_IN_1, self.MOTOR_3_IN_1_STATE)
        GPIO.output(self.MOTOR_3_IN_2, self.MOTOR_3_IN_2_STATE)

        # Setting up motor 4
        GPIO.setup(self.MOTOR_4, GPIO.OUT)
        GPIO.setup(self.MOTOR_4_IN_1, GPIO.OUT)
        GPIO.setup(self.MOTOR_4_IN_2, GPIO.OUT)
        GPIO.output(self.MOTOR_4_IN_1, self.MOTOR_4_IN_1_STATE)
        GPIO.output(self.MOTOR_4_IN_2, self.MOTOR_4_IN_2_STATE)

        # Setting up PWM Values
        self.MOTOR_1_PWM = GPIO.PWM(self.MOTOR_1, 100)
        self.MOTOR_2_PWM = GPIO.PWM(self.MOTOR_2, 100)
        self.MOTOR_3_PWM = GPIO.PWM(self.MOTOR_3, 100)
        self.MOTOR_4_PWM = GPIO.PWM(self.MOTOR_4, 100)


    def Pulse_Motor(self, motor):
        my_pwm = GPIO.PWM(motor, 100)
        my_pwm.start(self.POWER)
        time.sleep(self.WAIT_TIME)
        my_pwm.stop()

    def Engage_Motor(self, motor):
        if motor == 1:
            self.MOTOR_1_PWM.start(self.POWER)
        elif motor == 2:
            self.MOTOR_2_PWM.start(self.POWER)
        elif motor == 3:
            self.MOTOR_3_PWM.start(self.POWER)
        elif motor == 4:
            self.MOTOR_4_PWM.start(self.POWER)

    def Stop_Motor(self, motor):
        if motor == 1:
            self.MOTOR_1_PWM.stop()
        elif motor ==2:
            self.MOTOR_2_PWM.stop()
        elif motor ==3:
            self.MOTOR_3_PWM.stop()
        elif motor ==4:
            self.MOTOR_4_PWM.stop()

    def Z_Continuous(self, direction):
        """

        :param direction: (1 or 0) 0 for UPWARD direction, 1 for DOWNWARD direction
        :return: none
        """

        if self.MOTOR_1_ON:
            try:
                self.Stop_Motor(1)
                self.MOTOR_1_ON = 0
            except:
                pass

        if self.MOTOR_2_ON:
            try:
                self.Stop_Motor(2)
                self.MOTOR_2_ON = 0
            except:
                pass

        if self.DIRECTION not in (1,2):

            if not direction:
                # Configuring Motors for upward direction
                self.MOTOR_1_IN_1_STATE = 1
                self.MOTOR_1_IN_2_STATE = 0

                self.MOTOR_2_IN_1_STATE = 0
                self.MOTOR_2_IN_2_STATE = 1

            else:
                # Configuring Motors for downward direction
                self.MOTOR_1_IN_1_STATE = 0
                self.MOTOR_1_IN_2_STATE = 1

                self.MOTOR_2_IN_1_STATE = 1
                self.MOTOR_2_IN_2_STATE = 0

            # Setting states
            GPIO.output(self.MOTOR_1_IN_1, self.MOTOR_1_IN_1_STATE)
            GPIO.output(self.MOTOR_1_IN_2, self.MOTOR_1_IN_2_STATE)

            GPIO.output(self.MOTOR_2_IN_1, self.MOTOR_2_IN_1_STATE)
            GPIO.output(self.MOTOR_2_IN_2, self.MOTOR_2_IN_2_STATE)

            # Engaging motors
            self.Engage_Motor(1)
            self.Engage_Motor(2)

            self.MOTOR_1_ON = 1
            self.MOTOR_2_ON = 1

            self.DIRECTION = 1 + direction

        elif self.DIRECTION in (1,2):

            self.DIRECTION = 0

    def X_Continuous(self, direction):
        """

        :param direction: (1 or 0) 0 for FORWARD direction, 1 for BACKWARD direction
        :return: none
        """

        if self.MOTOR_3_ON:
            try:
                self.Stop_Motor(3)
                self.MOTOR_3_ON = 0
            except:
                pass

        if self.MOTOR_4_ON:
            try:
                self.Stop_Motor(4)
                self.MOTOR_4_ON = 0
            except:
                pass

        if self.DIRECTION not in (5, 6):

            if not direction:
                # Configuring Motors for upward direction
                self.MOTOR_3_IN_1_STATE = 1
                self.MOTOR_3_IN_2_STATE = 0

                self.MOTOR_4_IN_1_STATE = 0
                self.MOTOR_4_IN_2_STATE = 1

            else:
                # Configuring Motors for downward direction
                self.MOTOR_3_IN_1_STATE = 0
                self.MOTOR_3_IN_2_STATE = 1

                self.MOTOR_4_IN_1_STATE = 1
                self.MOTOR_4_IN_2_STATE = 0

            # Setting states
            GPIO.output(self.MOTOR_3_IN_1, self.MOTOR_3_IN_1_STATE)
            GPIO.output(self.MOTOR_3_IN_2, self.MOTOR_3_IN_2_STATE)

            GPIO.output(self.MOTOR_4_IN_1, self.MOTOR_4_IN_1_STATE)
            GPIO.output(self.MOTOR_4_IN_2, self.MOTOR_4_IN_2_STATE)

            # Engaging motors
            self.Engage_Motor(3)
            self.Engage_Motor(4)

            self.MOTOR_3_ON = 1
            self.MOTOR_4_ON = 1

            self.DIRECTION = 5 + direction

        elif self.DIRECTION in (5, 6):

            self.DIRECTION = 0

    def Yaw_Continuous(self, direction):
        """

        :param direction: (1 or 0) 0 for LEFT YAW direction, 1 for RIGHT YAW direction
        :return: none
        """

        if self.MOTOR_3_ON:
            try:
                self.Stop_Motor(3)
                self.MOTOR_3_ON = 0
            except:
                pass

        if self.MOTOR_4_ON:
            try:
                self.Stop_Motor(4)
                self.MOTOR_4_ON = 0
            except:
                pass

        if self.DIRECTION not in (3, 4):

            if not direction:
                # Configuring Motors for left yaw direction
                self.MOTOR_3_IN_1_STATE = 1
                self.MOTOR_3_IN_2_STATE = 0

                self.MOTOR_4_IN_1_STATE = 1
                self.MOTOR_4_IN_2_STATE = 0

            else:
                # Configuring Motors for right yaw direction
                self.MOTOR_3_IN_1_STATE = 0
                self.MOTOR_3_IN_2_STATE = 1

                self.MOTOR_4_IN_1_STATE = 0
                self.MOTOR_4_IN_2_STATE = 1

            # Setting states
            GPIO.output(self.MOTOR_3_IN_1, self.MOTOR_3_IN_1_STATE)
            GPIO.output(self.MOTOR_3_IN_2, self.MOTOR_3_IN_2_STATE)

            GPIO.output(self.MOTOR_4_IN_1, self.MOTOR_4_IN_1_STATE)
            GPIO.output(self.MOTOR_4_IN_2, self.MOTOR_4_IN_2_STATE)

            # Engaging motors
            self.Engage_Motor(3)
            self.Engage_Motor(4)

            self.MOTOR_3_ON = 1
            self.MOTOR_4_ON = 1

            self.DIRECTION = 3 + direction

        elif self.DIRECTION in (3, 4):

            self.DIRECTION = 0

    def Increase_Decrease_Power(self, inc):

        if inc:
            self.POWER += 5
        else:
            self.POWER -= 5

        if self.POWER > 100:
            self.POWER = 100

        elif self.POWER < 20:
            self.POWER = 20

        # self.MOTOR_1_PWM.stop()
        # self.MOTOR_2_PWM.stop()
        # self.MOTOR_3_PWM.stop()
        # self.MOTOR_4_PWM.stop()
        #
        # time.sleep(.01)
        #
        # if seosmf.MOTOR_1_ON:
        #     self.MOTOR_1_PWM.start(self.POWER)
        #
        # if self.MOTOR_2_ON:
        #     self.MOTOR_2_PWM.start(self.POWER)
        #
        # if self.MOTOR_3_ON:
        #     self.MOTOR_3_PWM.start(self.POWER)
        #
        # if self.MOTOR_4_ON:
        #     osmelf.MOTOR_4_PWM.start(self.POWER)


    # def Z_Movement(self, direction):
    #     """
    #
    #     :param direction: (1 or 0) 1 for UPWARD direction, 0 for DOWNWARD direction
    #     :return: none
    #     """
    #
    #
    #     if direction:
    #         # Configuring Motors for upward direction
    #         self.MOTOR_1_IN_1_STATE = 1
    #         self.MOTOR_1_IN_2_STATE = 0
    #
    #         self.MOTOR_2_IN_1_STATE = 0
    #         self.MOTOR_2_IN_2_STATE = 1
    #
    #     else:
    #         # Configuring Motors for downward direction
    #         self.MOTOR_1_IN_1_STATE = 0
    #         self.MOTOR_1_IN_2_STATE = 1
    #
    #         self.MOTOR_2_IN_1_STATE = 1
    #         self.MOTOR_2_IN_2_STATE = 0
    #
    #     # Setting states
    #     GPIO.output(self.MOTOR_1_IN_1, self.MOTOR_1_IN_1_STATE)
    #     GPIO.output(self.MOTOR_1_IN_2, self.MOTOR_1_IN_2_STATE)
    #
    #     GPIO.output(self.MOTOR_2_IN_1, self.MOTOR_2_IN_1_STATE)
    #     GPIO.output(self.MOTOR_2_IN_2, self.MOTOR_2_IN_2_STATE)
    #
    #     # Engaging motors
    #     pwm_1 = self.Engage_Motor(self.MOTOR_1)
    #     pwm_2 = self.Engage_Motor(self.MOTOR_2)
    #
    #     # Sleep time
    #     time.sleep(self.WAIT_TIME)
    #
    #     # Stopping motors
    #     self.Stop_Motor(pwm_1)
    #     self.Stop_Motor(pwm_2)


    def Execute(self, data):
        data = data.lower()
        if "w" in data:
            print("Go Forward")
            self.X_Continuous(1)
        elif "a" in data:
            print("Go Left")
            self.Yaw_Continuous(0)
        elif "d" in data:
            print("Go Right")
            self.Yaw_Continuous(1)
        elif "s" in data:
            print("Go Back")
            self.X_Continuous(0)
        elif "i" in data:
            print("Go Up")
            self.Z_Continuous(1)
        elif "k" in data:
            print("Go Down")
            self.Z_Continuous(0)
        elif "o" in data:
            print("Increase Power")
            self.Increase_Decrease_Power(1)
        elif "l" in data:
            print("Decrease Power")
            self.Increase_Decrease_Power(0)


if USE_CLIENT_SERVER:

    # Opening a socket to the server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('192.168.0.4', 10000)
    sock.connect(server_address)
    print("Connected to Server!")
    sock.send("Hello")

    # Creating an instance of the motors
    motors = Motors()

    # Reading in data from the socket
    try:
        while True:

            data = sock.recv(4096)
            # print("Received: " + data)
            motors.Execute(data)

    # Closing the socket
    except:
        sock.close()

    finally:
         GPIO.cleanup()

else:

    # Creating an instance of the motors
    motors = Motors()

    # Obtaining data from the user button press and executing motors
    try:
        while True:
            data = getch()
            motors.Execute(data)

            if "q" in data:
                break

    finally:

        try:
            motors.MOTOR_1_PWM.stop()
            motors.MOTOR_2_PWM.stop()
            motors.MOTOR_3_PWM.stop()
            motors.MOTOR_4_PWM.stop()
        except:
            pass

        GPIO.cleanup()
