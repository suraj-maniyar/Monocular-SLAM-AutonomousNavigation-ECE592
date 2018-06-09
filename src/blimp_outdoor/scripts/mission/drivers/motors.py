#!/usr/bin/env python
import rospy
import time

from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class Motors():
    def __init__(self):

	GPIO.setwarnings(False)

        # Right Motor B - CW (LOW / HIGH) Up/Down
        # self.motor1_pwm = 18
        # self.motor1_in1 = 24
        # self.motor1_in2 = 23

        # Motor A1 - LEFT Yaw Motor
        self.motor2_pwm = 0
        self.motor2_in1 = 5
        self.motor2_in2 = 6

        # self.motor2_in1 = 6
        # self.motor2_in2 = 5

        # Left Motor B - CW (HIGH / LOW) Up/Down
        # self.motor3_pwm = 13
        # self.motor3_in1 = 5
        # self.motor3_in2 = 6

        # Motor B1 - RIGHT Yay Motor
        self.motor4_pwm = 26
        # self.motor4_in1 = 19
        # self.motor4_in2 = 13

        self.motor4_in1 = 13
        self.motor4_in2 = 19

        GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.motor1_pwm, GPIO.OUT)
        GPIO.setup(self.motor2_pwm, GPIO.OUT)
        # GPIO.setup(self.motor3_pwm, GPIO.OUT)
        GPIO.setup(self.motor4_pwm, GPIO.OUT)
        # GPIO.setup(self.motor1_in1, GPIO.OUT)
        GPIO.setup(self.motor2_in1, GPIO.OUT)
        # GPIO.setup(self.motor3_in1, GPIO.OUT)
        GPIO.setup(self.motor4_in1, GPIO.OUT)
        # GPIO.setup(self.motor1_in2, GPIO.OUT)
        GPIO.setup(self.motor2_in2, GPIO.OUT)
        # GPIO.setup(self.motor3_in2, GPIO.OUT)
        GPIO.setup(self.motor4_in2, GPIO.OUT)

        # self.MOTOR_1_PWM = GPIO.PWM(self.motor1_pwm, 100)
        self.MOTOR_2_PWM = GPIO.PWM(self.motor2_pwm, 100)
        # self.MOTOR_3_PWM = GPIO.PWM(self.motor3_pwm, 100)
        self.MOTOR_4_PWM = GPIO.PWM(self.motor4_pwm, 100)

        # self.MOTOR_1_ON = False
        self.MOTOR_2_ON = False
        # self.MOTOR_3_ON = False
        self.MOTOR_4_ON = False

    """
    def DriveMotor1(self, speed):

        inv = speed < 0
        speed = min(100, abs(speed))
        if inv:
            GPIO.output(self.motor1_in1, GPIO.LOW)
            GPIO.output(self.motor1_in2, GPIO.HIGH)
        else:
            GPIO.output(self.motor1_in1, GPIO.HIGH)
            GPIO.output(self.motor1_in2, GPIO.LOW)

        if self.MOTOR_1_ON:
            self.MOTOR_1_PWM.ChangeDutyCycle(abs(speed))
        else:
            self.MOTOR_1_PWM.start(abs(speed))

        self.MOTOR_1_ON = True
    """

    def DriveMotor2(self, speed):

        inv = speed < 0
        speed = min(100, abs(speed))
        if inv:
            GPIO.output(self.motor2_in1, GPIO.LOW)
            GPIO.output(self.motor2_in2, GPIO.HIGH)
        else:
            GPIO.output(self.motor2_in1, GPIO.HIGH)
            GPIO.output(self.motor2_in2, GPIO.LOW)

        if self.MOTOR_2_ON:
            self.MOTOR_2_PWM.ChangeDutyCycle(abs(speed))
        else:
            self.MOTOR_2_PWM.start(abs(speed))

        self.MOTOR_2_ON = True

    """
    def DriveMotor3(self, speed):
        inv = speed < 0
        speed = min(100, abs(speed))

        if inv:
            GPIO.output(self.motor3_in1, GPIO.LOW)
            GPIO.output(self.motor3_in2, GPIO.HIGH)
        else:
            GPIO.output(self.motor3_in1, GPIO.HIGH)
            GPIO.output(self.motor3_in2, GPIO.LOW)

        if self.MOTOR_3_ON:
            self.MOTOR_3_PWM.ChangeDutyCycle(abs(speed))
        else:
            self.MOTOR_3_PWM.start(abs(speed))

        self.MOTOR_3_ON = True
    """

    def DriveMotor4(self, speed):
        inv = speed < 0
        speed = min(100, abs(speed))
        if inv:
            GPIO.output(self.motor4_in1, GPIO.LOW)
            GPIO.output(self.motor4_in2, GPIO.HIGH)
        else:
            GPIO.output(self.motor4_in1, GPIO.HIGH)
            GPIO.output(self.motor4_in2, GPIO.LOW)

        if self.MOTOR_4_ON:
            self.MOTOR_4_PWM.ChangeDutyCycle(abs(speed))
        else:
            self.MOTOR_4_PWM.start(abs(speed))

        self.MOTOR_4_ON = True

    def turn(self, speed):
        speed = int(speed)
        self.DriveMotor2(-speed)
        self.DriveMotor4(speed)

    def forward(self):
        speed = 100
        self.DriveMotor2(speed)
        self.DriveMotor4(speed)

    def forward_stop(self):
        self.StopMotors24()

    def backward(self):
        speed = 100
        self.DriveMotor2(-speed)
        self.DriveMotor4(-speed)

    def backward_stop(self):
        self.StopMotors24()

    """
    def upward(self):
        speed = 100
        self.DriveMotor1(speed)
        self.DriveMotor3(speed)

    def upward_stop(self):
        self.StopMotors13()

    def downward(self):
        speed = 100
        self.DriveMotor1(-speed)
        self.DriveMotor3(-speed)

    def downward_stop(self):
        self.StopMotors13()


    def StopMotors13(self):

        if self.MOTOR_1_ON:
            self.MOTOR_1_PWM.stop()
            self.MOTOR_1_ON = False
        if self.MOTOR_3_ON:
            self.MOTOR_3_PWM.stop()
            self.MOTOR_3_ON = False
    """


    def StopMotors24(self):

        if self.MOTOR_2_ON:
            self.MOTOR_2_PWM.stop()
            self.MOTOR_2_ON = False
        if self.MOTOR_4_ON:
            self.MOTOR_4_PWM.stop()
            self.MOTOR_4_ON = False


if __name__ == "__main__":
	print("In main")
	a = Motors()
	a.forward()
	time.sleep(5)
	a.StopMotors24()


