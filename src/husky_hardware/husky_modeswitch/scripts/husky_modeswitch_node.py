#!/usr/bin/env python

# this node muxes the control info from a joystick and auto control from husky_controller into a singletopic /husky_controlinfo for the hardware node to control the modules

import rospy
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Pose,PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from husky_modeswitch.msg import *
from husky_controller.msg import *
import time
import rosbag

global path_pub,path,poses,odom,prev_odom,seq
global prevbutton,pub,mode
global xlin_man,zang_man,pan_man,tilt_man
global xlin_auto,zang_auto,pan_auto,tilt_auto
global STOPGO,prevbutton_STOPGO
global move_list,move_list_index,move_list_index_reverse
global Speed_Limit
global Record
global speed


def save_path():
    global path_pub,path,poses,odom,prev_odom,seq
    filename = time.strftime("path_wheel_encoder_%Y_%m_%d__%H_%M_%S.bag")
    try:
        bag=rosbag.Bag(filename,'w')
        bag.write('/odom/path',path)
        bag.close()
        poses=[]
        print "path saved to :" + filename
    except:
        print "error storing"
#-----------------------------------------------------------------------
def callback_odom(data):
    global path_pub,path,poses,odom,prev_odom,seq
    odom=data
    if (abs(odom.pose.pose.position.x-prev_odom.pose.pose.position.x) > 0.01) or (abs(odom.pose.pose.position.y-prev_odom.pose.pose.position.y) > 0.01):
        pose_stamped=PoseStamped()
        pose_stamped.header.stamp=rospy.Time.now()
        pose_stamped.header.frame_id='odom'
        pose_stamped.header.seq=seq
        seq+=1
        pose_stamped.pose=odom.pose.pose
        poses.append(pose_stamped)
        path.poses=poses
        path.header.stamp=rospy.Time.now()
        path_pub.publish(path)
    prev_odom=odom
#-----------------------------------------------------------------------
def movebase_callback(data):
    global xlin_auto,zang_auto
    xlin_auto=data.linear.x
    zang_auto=data.angular.z
    print "update: "+str(xlin_auto)+"\t"+str(zang_auto)

#-----------------------------------------------------------------------
def velocity_callback(data):
    global xlin_auto,zang_auto,pan_auto,tilt_auto
    #rospy.loginfo(rospy.get_caller_id() + '%s\n%s', data.xlin_vel,data.zang_vel)
    xlin_auto=data.linear.x
    zang_auto=data.angular.z
    print "update: "+str(xlin_auto)+"\t"+str(zang_auto)
#-----------------------------------------------------------------------
def joystick_callback(data):
    global xlin_man,zang_man,mode,pan_man,tilt_man
    global prevbutton
    #rospy.loginfo(rospy.get_caller_id() + '%s\n%s', data.axes,data.buttons)
    global STOPGO,mode,Auto_Reverse,Speed_Limit
    global path

    button_pressed = False

    if data.buttons[2]==1 and prevbutton.buttons[2]==0:
        button_pressed = True
        if Speed_Limit == 1:
            print "Speed Limit Cleared"
            Speed_Limit=0
        else:
            print "Speed Limit Set"
            Speed_Limit=1

#auto mode - button 0

    if data.buttons[0] == 1 and prevbutton.buttons[0] == 0:
        button_pressed = True
        STOPGO = 1

    if data.buttons[1] == 1 and prevbutton.buttons[1] == 0:
        button_pressed = True
        STOPGO = 0

    if data.buttons[5] == 1 and prevbutton.buttons[5] == 0:
        button_pressed = True
        save_path()
        print "Recording saved"



    if data.buttons[3] == 1 and prevbutton.buttons[3] == 0:
        button_pressed = True
        if mode == 1:
            mode=0
        else:
            mode=1
    #get joystick data:
    if button_pressed:
        if Speed_Limit:
            print "Speed_Limit is Set\t",
        else:
            print "Speed_Limit is Cleared\t",

        if mode == 0:
            print "mode : Manual",
        else:
            print "mode : Auto",
        if STOPGO:
            print "\tGO"
        else:
            print "\tSTOP"

    xlin_man=data.axes[1]
    zang_man=data.axes[0]
    pan_man=data.axes[4]
    tilt_man=data.axes[3]
    prevbutton=data


#-----------------------------------------------------------------------
def clipValue(value,bound):
    if abs(value) > bound:
        if value > 0:
            value=bound
        else:
            value=-bound
    return value

#-----------------------------------------------------------------------

def husky_modeswitchloop():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'husky_modeswitchloop' node so that multiple loops can
    # run simultaneously.
    global xlin_man,zang_man,pan_man,tilt_man
    global xlin_auto,zang_auto,pan_auto,tilt_auto
    global pub,mode
    global STOPGO,prevbutton_STOPGO
    global Speed_Limit
    global path_pub,path,poses,odom,prev_odom,seq
    global Record,prevbutto
    global speed
    speed = 0.1
    Record=0
    seq=1
    poses=[]
    odom=Odometry()
    prev_odom=Odometry()
    path=Path()
    path.header.frame_id='odom'

    Speed_Limit=0

    STOPGO=0
    xlin_man=0
    zang_man=0
    pan_man=0
    tilt_man=0
    xlin_auto=0
    zang_auto=0
    pan_auto=0
    tilt_auto=0
    prevbutton=Joy()
    prevbutton.buttons=[0]*10
    prevbutton.axes=[0]*10
    mode=0
    print "STOP"
    print "Manual"

    pub = rospy.Publisher('/husky_modeswitch/controlhardware_interface', husky_controlinfo,queue_size=10)
    rospy.init_node('husky_modeswitch', anonymous=True)
    rospy.Subscriber('/joy', Joy , joystick_callback)
    rospy.Subscriber('/husky_controller/velocitycontrol', Twist , velocity_callback)
    rospy.Subscriber('/cmd_vel',Twist, movebase_callback)

    path.header.frame_id='odom'
    path.header.stamp=rospy.Time.now()
    odom_topic=rospy.get_param('odom_topic_name','/odom')
    print odom_topic
    rospy.Subscriber(odom_topic,Odometry,callback_odom)
    path_pub=rospy.Publisher('/odom/path',Path,queue_size=10)

    rate=rospy.Rate(10)
    xlin=0
    zang=0
    pan=0
    tilt=0
    while not rospy.is_shutdown():
        if STOPGO == 1:
                if mode == 1:
                    xlin=xlin_auto
                    zang=zang_auto
                    pan=0
                    tilt=0

                else:
                    if Speed_Limit:
                        xlin=speed*xlin_man
                        zang=speed*zang_man
                    else:
                        xlin=xlin_man
                        zang=zang_man
                    pan=pan_man
                    tilt=tilt_man
        else:
            xlin=0
            zang=0
            pan=0
            tilt=0

        xlin=clipValue(xlin,1)
        zang=clipValue(zang,1)
        pub.publish(mode,xlin,zang,pan,tilt)
        rate.sleep()


if __name__ == '__main__':
    husky_modeswitchloop()
