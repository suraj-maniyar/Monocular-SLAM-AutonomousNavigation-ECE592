#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import time
import RPi.GPIO as GPIO

def keys():
    pub = rospy.Publisher('ultrasonic',Float64, queue_size=10)
    #pub1 = rospy.Publisher('key1',Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        distance1 = measure_average(s1)    
        #distance2 = measure_average(s2)    
        
        rospy.loginfo(distance1)
        #rospy.loginfo(distance2)
        pub.publish(distance1)
        #pub1.publish(distance2)
        #rate.sleep()

def measure(x):
    GPIO.output(x, True)
    time.sleep(0.00001)
    GPIO.output(x, False)
    start = time.time()

    # set line to input to check for start of echo response
    GPIO.setup(x, GPIO.IN)
    while GPIO.input(x)==0:
        start = time.time()
    # Wait for end of echo response
    while GPIO.input(x)==1:
        stop = time.time()

    GPIO.setup(x, GPIO.OUT)
    GPIO.output(x, False)

    elapsed = stop-start
    distance = (elapsed * 34300)/2.0
    return distance

def measure_average(x):
     distance=measure(x)
     time.sleep(0.1)
     return distance


if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    print "Ultrasonic Measurement"
    s1=16
    #s2=12
    GPIO.setup(s1,GPIO.OUT)
    #GPIO.setup(s2,GPIO.OUT)
  
    GPIO.output(s1, False)
    #GPIO.output(s2, False)

    try:
        keys()
    except rospy.ROSInterruptException:
        pass