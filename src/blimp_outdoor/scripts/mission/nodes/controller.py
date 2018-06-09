#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  22 10:44:07 2018

@author: vijayaganesh
"""
import rospy,sys
sys.path.append('/home/ubuntu/catkin_ws/src/outdoor_blimp/scripts/mission/lib')
sys.path.append('/home/ubuntu/catkin_ws/src/outdoor_blimp/scripts/mission/drivers')
sys.path.append('/home/ubuntu/catkin_ws/src/outdoor_blimp/scripts/mission/services')
sys.path.append('/home/ubuntu/catkin_ws/src/outdoor_blimp/scripts/mission')
from attitude_control import Attitude_Control

class Controller:
  def __init__(self):
    rospy.init_node('Outdoor_Blimp_Controller')
    self.attitude_control = Attitude_Control()
  
  def run(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.attitude_control.control_loop()
      #self.attitude_control.pan_tilt()
      rate.sleep()
  
if __name__ == '__main__':
  c = Controller()
  c.run()
