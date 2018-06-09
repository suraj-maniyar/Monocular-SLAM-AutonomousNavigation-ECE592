#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  22 10:44:07 2018

"""
import rospy,sys
sys.path.append('/home/ubuntu/catkin_ws/src/yaw_cntrl/src')
sys.path.append('/home/ubuntu/catkin_ws/src/yaw_cntrl/src')
sys.path.append('/home/ubuntu/catkin_ws/src/yaw_cntrl/src')
sys.path.append('/home/ubuntu/catkin_ws/src/yaw_cntrl/src')
from Yaw_control import Yaw_control
from  velocity_control import Vel_control
class Controller:
  def __init__(self):
    rospy.init_node('Indoor_Blimp_Controller')
    self.vel_control=Vel_control()
  def run(self):
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
      rate.sleep()
  
if __name__ == '__main__':
  c = Controller()
  c.run()
