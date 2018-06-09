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
print(sys.path)
from blimp_publisher import Blimp_Publisher

class Mission:
  def __init__(self):
    rospy.init_node('Outdoor_Blimp_Publisher')
    self.blimp_pub = Blimp_Publisher()
  
  def run(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.blimp_pub.publish_imu()
#      self.blimp_pub.publish_cam_imu()
#      self.blimp_pub.publish_gps()
      rate.sleep()

if __name__ == '__main__':
  m = Mission()
  m.run()
