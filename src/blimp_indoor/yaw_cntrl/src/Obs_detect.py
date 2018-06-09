#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  14 11:31:17 2018
@author: adityarasam
"""

import rospy
from blimp import Blimp
from std_msgs.msg import Float64

class obstacle_check:
  def __init__(self):
    rospy.Subscriber("/ultrasonic",Float64,self.obstacle_control)
    self.blimp = Blimp()
    
  def obstacle_control(self,data):
    U=int(data.data)
    #print(U)
    if U<70:
      print("entered if loop")
      self.blimp.motors.forward_stop()
      self.blimp.motors.backward_stop()
      self.blimp.motors.upward_stop()    
      self.blimp.motors.downward_stop()       
    else:
       print("otside loop")    
      