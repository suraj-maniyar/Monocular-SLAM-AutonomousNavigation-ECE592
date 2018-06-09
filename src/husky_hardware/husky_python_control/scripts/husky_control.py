#!/usr/bin/python

from clearpath.horizon import Horizon
from clearpath.horizon.transports import Serial
from clearpath import utils
import rospy
from geometry_msgs.msg import Twist, Pose, Vector3,Quaternion, Point
from nav_msgs.msg import Odometry
from husky_controller.msg import velocities
import tf
import math
import time
from std_msgs.msg import String
from husky_modeswitch.msg import *
import socket
import cPickle as pickle

def encoder_data_handler(payload, timestamp):
# payload[0] is the left wheel travel
# payload[1] is the right wheel travel
# payload[2] is the left wheel instantaneous speed
# payload[3] is the right wheel instantaneous speed

    global Husky_x_pos, Husky_y_pos, Husky_heading
    global Husky_prev_L_wheel, Husky_prev_R_wheel
    global firstRead

    if firstRead == True:
        Husky_prev_L_wheel = payload[0]
        Husky_prev_R_wheel = payload[1]
        firstRead=False

    #Average of vehicles' left and right wheels
    displacement = ((payload[0]-Husky_prev_L_wheel)+(payload[1]-Husky_prev_R_wheel))/2
    #Vehicle track /wheel seperation is 0.54 meters
    rotation = ((payload[1]-Husky_prev_R_wheel)-(payload[0]-Husky_prev_L_wheel))/0.54
    rotation = rotation/2
    Husky_heading = Husky_heading + rotation  #Have to account for greater than 2PI angles
    if rotation == 0:
        #radius =  displacement
        Husky_x_pos = Husky_x_pos + displacement * math.cos(Husky_heading)
        Husky_y_pos = Husky_y_pos + displacement * math.sin(Husky_heading)
    else:
        #radius =  displacement / rotation
        if Husky_heading > 2*math.pi:
            Husky_heading = Husky_heading - 2 *math.pi
        if Husky_heading < 0:
            Husky_heading = Husky_heading + 2 *math.pi
        #Husky_x_pos = Husky_x_pos + (radius - radius*math.cos(Husky_heading))
        #Husky_y_pos = Husky_y_pos + radius*math.sin(Husky_heading)
        Husky_x_pos = Husky_x_pos + (displacement * math.cos(Husky_heading))
        Husky_y_pos = Husky_y_pos + (displacement * math.sin(Husky_heading))

    Husky_prev_L_wheel = payload[0]
    Husky_prev_R_wheel = payload[1]
    #print(payload.print_format())
    #pub.publish(payload)
    current_time = rospy.Time.now()

    #Form a quaternion transform from Yaw/Husky_heading values
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, Husky_heading)
    odom_broadcaster = tf.TransformBroadcaster()
    #Publish the transform over tf
    odom_broadcaster.sendTransform(
        (Husky_x_pos, Husky_y_pos, 0),
         odom_quat,
         current_time,
         "base_link",
         "odom"
    )

    odom = Odometry()
    odom.header.stamp= current_time
    odom.header.frame_id= "odom"
    odom.pose.pose = Pose(Point(Husky_x_pos, Husky_y_pos, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(payload[2], payload[3], 0), Vector3(0, 0, 0))
    print Husky_x_pos, Husky_y_pos, Husky_heading
    print "Time: " + str(current_time)
    # publish the message
    Odometry_publisher.publish(odom)


def callback(data):

    global conn
    #set_velocity(self, trans = 0.0, rot = 0.0, accel = 0.0)
    #print "Translation = " + str(data.linear.x)
    #print "Rotation = " + str(data.angular.z)
    # data.linear.x*=0.7    #The scaling is because the analog stick variation is a sine
    # data.angular.z*=0.7   #function and goes upto 1.414 when pointed NW,NE,SE and SW
    data.linear.x=math.floor(data.linear.x*1000)/1000
    data.angular.z=math.floor(data.angular.z*1000)/1000

    #print "Left wheel" + str(data.linear.x-data.angular.z)
    #print "Right wheel" + str(data.linear.x+data.angular.z)
    data_to_send=[data.linear.x,data.angular.z]
    #print data_to_send
    string_format_data = pickle.dumps(data_to_send)
    conn.sendall(string_format_data)
    # Husky.set_differential_speed(left_speed= data.linear.x-data.angular.z, right_speed= data.linear.x+data.angular.z,
    #                               left_accel=0.5, right_accel=0.5)

def motor_control_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + '%s\t%s\t%s\t%s\t%s', data.auto,data.xlin_vel,data.zang_vel,data.pan_angle,data.tilt_angle)
    global hwdpub
    twistData=Twist(Vector3(data.xlin_vel,0.0,0.0),Vector3(0.0,0.0,data.zang_vel))
    hwdpub.publish(twistData)

if __name__ == '__main__':

    global Husky, Odometry_publisher, Husky_x_pos, Husky_y_pos, Husky_heading
    global Husky_prev_L_wheel,Husky_prev_R_wheel
    global firstRead, hwdpub, conn

    rospy.init_node('Husky_movement', anonymous=True)
    TCP_flag = rospy.get_param('~TCP','True')
#Get parameteres from command line while running the node. TCP socket will run by default
#Example to pass parameters : rosrun husky_python_control husky_control.py _TCP:=False/True

    firstRead=True
    Husky_x_pos = 0
    Husky_y_pos = 0
    Husky_heading = 0
    Husky_prev_L_wheel = 0
    Husky_prev_R_wheel = 0

    TCP_IP = '10.42.0.1'    # BBB/Pi IP address on Ethernet Input of Router
    TCP_PORT = 12345
    BUFFER_SIZE = 1024

    if TCP_flag:
        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        conn.connect((TCP_IP, TCP_PORT))
        conn.settimeout(0.1)
        #Subscribe to Husky movement commands if TCP is not disabled
        rospy.Subscriber("/joy_teleop/cmd_vel", Twist, callback)

    #Publish Odometry commands
    Odometry_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
    hwdpub = rospy.Publisher('/joy_teleop/cmd_vel',Twist,queue_size=10)
    rospy.Subscriber('/husky_modeswitch/controlhardware_interface',husky_controlinfo,motor_control_callback)

    if TCP_flag:
        while True:

            try:
                data_received_string = conn.recv(BUFFER_SIZE)
                data_received = pickle.loads(data_received_string)
                payload = [data_received[0], data_received[1], data_received[2], data_received[3]]
                timestamp = data_received[4]
                encoder_data_handler(payload, timestamp)

            except socket.timeout:
                continue
    rospy.spin()
