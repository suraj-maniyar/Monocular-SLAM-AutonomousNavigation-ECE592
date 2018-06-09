#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  1 16:10:20 2018

@author: vijayaganesh
"""

from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Pose, TwistStamped, Vector3
from outdoor_blimp.msg import gps_info, gps_velocity
from std_msgs.msg import Header
import rospy
import serial
import time
from motors import Motors

class Blimp:
  """
  Principle Class of Blimp
  """
  def __init__(self):
    self.motors = Motors()
    
  
