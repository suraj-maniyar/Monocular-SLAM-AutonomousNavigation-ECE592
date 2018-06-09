#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  14 11:31:17 2018
@author: adityarasam
"""

import rospy
from geometry_msgs.msg import PoseStamped,Vector3
from pid_controller import PID_Controller
from tf.transformations import euler_from_quaternion
from blimp import Blimp
from sensor_msgs.msg import Imu

class Yaw_control:
  def __init__(self):
    rospy.Subscriber("/Indoor_Bldddimp/setpoint_position",PoseStamped,self.process_setpoint)
    rospy.Subscriber("/imu/data",Imu,self.process_current)
    self.blimp = Blimp()
    self.set_yaw = 90
    self.set_trans_x = 0
    self.set_trans_z = 0
    
    self.curr_yaw = 0
    self.curr_trans_x = 0
    self.curr_trans_z = 0
    
    yaw_kp = 0.8  #rospy.get_param("~yaw_kp")
    yaw_ki = 0.8 #rospy.get_param("~yaw_ki")
    yaw_kd = 0.8 #rospy.get_param("~yaw_kd")
    
    print("BBBBBBBBBBBBBBBBBBBB")
    self.yaw_control =  PID_Controller(yaw_kp,yaw_ki,yaw_kd)
    print("AAAAAAAAAAAAAAAAAAAAA")
    
  
  
  def control_loop(self):

    print("Set Yaw Angle: "+repr(self.set_yaw))
    print("Current Yaw Angle: "+repr(self.curr_yaw))
    mv = self.yaw_control.corrective_action(self.set_yaw,self.curr_yaw)
    print("correctedspeed")
    print(mv)
    
    
    self.blimp.motors.turn(mv)
        
  def pan_tilt(self):
    self.pan_tilt.pitch(10)
    self.pan_tilt.roll(6)
  
  def process_setpoint(self,poseStamped):
    
    pose = poseStamped.pose
    quart = pose.orientation
    trans = pose.position
    exp_quart = [quart.x,quart.y,quart.z,quart.w]
    r,p,y = euler_from_quaternion(exp_quart)

    self.set_yaw = y
    
  def process_current(self,imu_data):
    
    self.curr_yaw =imu_data.orientation.x
    
