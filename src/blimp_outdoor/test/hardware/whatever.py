#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# Right INNNER Side - CW (LOW / HIGH)
motor1_pwm = 12
motor1_in1 = 16
motor1_in2 = 20

# Right OUTER Side - CW (LOW / HIGH)
motor2_pwm = 18
motor2_in1 = 23
motor2_in2 = 24

# Left OUTER Side - CW (HIGH / LOW)
motor3_pwm = 13
motor3_in1 = 5
motor3_in2 = 6

# Left INNER Side - CW (HIGH / LOW)
motor4_pwm = 17
motor4_in1 = 27
motor4_in2 = 22

global MOTOR_1_PWM
global MOTOR_2_PWM
global MOTOR_3_PWM
global MOTOR_4_PWM


def DriveMotor1(inv):
    global MOTOR_1_PWM
    if inv:
        GPIO.output(motor1_in1, GPIO.LOW)
        GPIO.output(motor1_in2, GPIO.HIGH)
    else:
        GPIO.output(motor1_in1, GPIO.HIGH)
        GPIO.output(motor1_in2, GPIO.LOW)

    MOTOR_1_PWM.start(100)


def DriveMotor2(inv):
    global MOTOR_2_PWM
    if inv:
        GPIO.output(motor2_in1, GPIO.LOW)
        GPIO.output(motor2_in2, GPIO.HIGH)
    else:
        GPIO.output(motor2_in1, GPIO.HIGH)
        GPIO.output(motor2_in2, GPIO.LOW)

    MOTOR_2_PWM.start(100)


def DriveMotor3(inv):
    global MOTOR_3_PWM
    if inv:
        GPIO.output(motor3_in1, GPIO.LOW)
        GPIO.output(motor3_in2, GPIO.HIGH)
    else:
        GPIO.output(motor3_in1, GPIO.HIGH)
        GPIO.output(motor3_in2, GPIO.LOW)

    MOTOR_3_PWM.start(100)


def DriveMotor4(inv):
    global MOTOR_4_PWM
    if inv:
        GPIO.output(motor4_in1, GPIO.LOW)
        GPIO.output(motor4_in2, GPIO.HIGH)
    else:
        GPIO.output(motor4_in1, GPIO.HIGH)
        GPIO.output(motor4_in2, GPIO.LOW)

    MOTOR_4_PWM.start(100)


def StopMotors():
    global MOTOR_1_PWM
    global MOTOR_2_PWM
    global MOTOR_3_PWM
    global MOTOR_4_PWM
    MOTOR_1_PWM.stop()
    MOTOR_2_PWM.stop()
    MOTOR_3_PWM.stop()
    MOTOR_4_PWM.stop()


def drive_motor(data):
    global MOTOR_1_PWM
    global MOTOR_2_PWM
    global MOTOR_3_PWM
    global MOTOR_4_PWM
    print("Inside Drive Motor")
    if (data.linear.x > 0):

        # UP ARROW
        # Forward
        # Motor 2 CW
        DriveMotor2(1)

        # Motor 3 CCW
        DriveMotor3(0)


    elif (data.linear.x < 0):

        # DOWN ARROW
        # Up
        # Motor 1 CW
        DriveMotor1(1)

        # Motor 4 CCW
        DriveMotor4(0)

    else:
        MOTOR_1_PWM.stop()
        MOTOR_4_PWM.stop()

    if (data.angular.z > 0):

        # LEFT ARROW
        # Left
        # Motor 2 CW
        DriveMotor2(1)

        # Motor 3 CW
        DriveMotor3(1)

    elif (data.angular.z < 0):

        # RIGHT ARROW
        # Right
        # Motor 2 CCW
        DriveMotor2(0)

        # Motor 3 CCW
        DriveMotor3(0)

    else:
        pass


# StopMotors()


def listener():
    rospy.init_node('Blimp_drive_motors', anonymous=True)
    rospy.Subscriber("Blimp_Commands", Twist, drive_motor)
    rospy.spin()


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(motor1_pwm, GPIO.OUT)
    GPIO.setup(motor2_pwm, GPIO.OUT)
    GPIO.setup(motor3_pwm, GPIO.OUT)
    GPIO.setup(motor4_pwm, GPIO.OUT)
    GPIO.setup(motor1_in1, GPIO.OUT)
    GPIO.setup(motor2_in1, GPIO.OUT)
    GPIO.setup(motor3_in1, GPIO.OUT)
    GPIO.setup(motor4_in1, GPIO.OUT)
    GPIO.setup(motor1_in2, GPIO.OUT)
    GPIO.setup(motor2_in2, GPIO.OUT)
    GPIO.setup(motor3_in2, GPIO.OUT)
    GPIO.setup(motor4_in2, GPIO.OUT)

    MOTOR_1_PWM = GPIO.PWM(motor1_pwm, 100)
    MOTOR_2_PWM = GPIO.PWM(motor2_pwm, 100)
    MOTOR_3_PWM = GPIO.PWM(motor3_pwm, 100)
    MOTOR_4_PWM = GPIO.PWM(motor4_pwm, 100)

    listener()
