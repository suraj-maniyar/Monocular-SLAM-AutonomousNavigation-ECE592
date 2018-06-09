#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  4 18:39:17 2018

@author: vijayaganesh
"""

import rospy
from geometry_msgs.msg import PoseStamped,Vector3
from pid_controller import PID_Controller
from tf.transformations import euler_from_quaternion
from blimp import Blimp
# from actuate_servos import PanTilt
import pigpio
import numpy as np
class Attitude_Control:
  def __init__(self):
    rospy.Subscriber("/Outdoor_Blimp/setpoint_position",PoseStamped,self.process_setpoint)
    rospy.Subscriber("/Outdoor_Blimp/imu/orientation",Vector3,self.process_current)
    self.blimp = Blimp()
    self.set_yaw = 0
    self.set_pitch = 0
    self.set_roll = 0
    self.set_trans_x = 0
    self.set_trans_z = 0
    self.curr_pitch = 0
    self.curr_roll = 0
    self.curr_yaw = 0
    self.curr_trans_x = 0
    self.curr_trans_z = 0
    
    yaw_kp = rospy.get_param("~yaw_kp")
    yaw_ki = rospy.get_param("~yaw_ki")
    yaw_kd = rospy.get_param("~yaw_kd")
    
    trans_x_kp = rospy.get_param("~trans_x_kp")
    trans_x_ki = rospy.get_param("~trans_x_ki")
    trans_x_kd = rospy.get_param("~trans_x_kd")
    
    trans_z_kp = rospy.get_param("~trans_z_kp")
    trans_z_ki = rospy.get_param("~trans_z_ki")
    trans_z_kd = rospy.get_param("~trans_z_kd")

    self.curr_roll = 0
    
    self.yaw_control =  PID_Controller(yaw_kp,yaw_ki,yaw_kd)
    
    self.trans_x_control = PID_Controller(trans_x_kp,trans_x_ki,trans_x_kd)
    
    self.trans_z_control = PID_Controller(trans_z_kp,trans_z_ki,trans_z_kd)

    self.servos = pigpio.pi()
    
    # self.pan_tilt = PanTilt(8, 25)
  
  
  def control_loop(self):

    # print("Set Yaw Angle: "+ repr(self.set_yaw))
    # print("Current Yaw Angle: "+repr(self.curr_yaw))
    mv = self.yaw_control.corrective_action(self.set_yaw,self.curr_yaw)
    self.blimp.motors.turn(-mv)
    # self.blimp.motors.downward()
    self.trans_x_control.corrective_action(self.set_trans_x,self.curr_trans_x)
    self.trans_z_control.corrective_action(self.set_trans_z,self.curr_trans_z)
    # self.pan_tilt()
    
  def pan_tilt(self):

    roll = self.curr_roll - self.set_roll
    pitch = self.curr_pitch - self.set_pitch
    
    servo_pitch_val = int(1600+8.88*(pitch))
    servo_roll_val = int(1250 + 9.44 * roll + 0.0123 * roll**2)
    
    # print("Roll Val: %5d" % servo_roll_val)
    # print("Pitch Val %5d" % servo_pitch_val)
    
    servo_roll_val = min(servo_roll_val, 2250)
    servo_roll_val = max(servo_roll_val, 500)

    servo_pitch_val = min(servo_pitch_val, 2500)
    servo_pitch_val = max(servo_pitch_val, 800)
  
    self.servos.set_servo_pulsewidth(24, servo_roll_val)
    self.servos.set_servo_pulsewidth(23, servo_pitch_val)
    # self.pan_tilt.roll(6)
  
  def process_setpoint(self,poseStamped):
    
    pose = poseStamped.pose
    quart = pose.orientation
    trans = pose.position
    #exp_quart = [quart.x,quart.y,quart.z,quart.w]
    #r,p,y = euler_from_quaternion(exp_quart)
    self.set_trans_x = 0
    self.set_trans_z = 0
    self.set_yaw = quart.x
    self.set_pitch = 0
    self.set_roll = 0
    #print("Set Yaw Yaw Angle: "+repr(y))
    
  def process_current(self,orientation):
    
    #pose = poseStamped.pose
    #quart = pose.orientation
    #trans = pose.position
    #exp_quart = [quart.x,quart.y,quart.z,quart.w]
    #r,p,y = euler_from_quaternion(exp_quart)
    self.curr_trans_x = 0
    self.curr_trans_z = 0
    self.curr_yaw = orientation.x
    self.curr_pitch = orientation.y
    self.curr_roll = orientation.z
    #print("Current Yaw Angle: "+repr(y))
    
  
  
