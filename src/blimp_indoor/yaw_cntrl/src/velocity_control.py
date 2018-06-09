#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  14 11:31:17 2018
@author: adityarasam
"""

import rospy
from geometry_msgs.msg import PoseStamped,Vector3
from pid_controller import PID_Controller
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float64
import time
from tf.transformations import euler_from_quaternion
from blimp import Blimp
from sensor_msgs.msg import Imu

class Vel_control:
  def __init__(self):
    
    rospy.Subscriber("key",String,self.velocity_set)



    rospy.Subscriber("/ultrasonic",Float64,self.obstacle_control)


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
    
    self.yaw_control =  PID_Controller(yaw_kp,yaw_ki,yaw_kd)
    
  
  def velocity_set(self,data):
    G=data.data
    G = G.split("-")
    print("G",G)
    velocity_SP = float(G[1])
    velocity_dir= G[0]
    


    
    if velocity_dir=="F":
         self.blimp.motors.forward(velocity_SP)
    if velocity_dir=="B":
         self.blimp.motors.backward(velocity_SP)
  
    if velocity_dir=="U":
         #time.sleep(2)
         self.blimp.motors.upward(velocity_SP)
         print("up:",G)
         #time.sleep(2)

    if velocity_dir=="E":
         #time.sleep(2)

         self.blimp.motors.downward(velocity_SP)
         #time.sleep(2)
         
    if velocity_dir=="S":
        self.blimp.motors.forward_stop()
        self.blimp.motors.backward_stop()
    if velocity_dir=="Q":        
        self.blimp.motors.upward_stop()    
        self.blimp.motors.downward_stop()   
    
  def obstacle_control(self,data):
    U=int(data.data)
    #print(U)


    if U<70:
        Obs_ahead = True
        print("entered if loop")
        self.blimp.motors.forward_stop()
        self.blimp.motors.backward_stop()
        self.blimp.motors.upward_stop()    
        self.blimp.motors.downward_stop()       
    else:
         Obs_ahead = False
         print("otside loop")    









