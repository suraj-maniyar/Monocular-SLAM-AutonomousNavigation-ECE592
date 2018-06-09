#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  30 16:37:34 2018

@author: vijayaganesh
"""

#!/usr/bin/env python
import rospy

import serial

class GPS:
  @staticmethod
  def parseGPS(data):
      if data[0:6] == "$GPGGA":
        s = data.split(",")
        if s[6] == "0" or date_flag!=1 :
            print "no satellite data available"
            return
        dirLat = s[3]
        dirLon = s[5]
        
        alt_ellipsoid = float(s[9])
        status = int(s[6])
        if dirLat == "S":
          lat= (-GPS.decode(s[2])) 
        else:
          lat= GPS.decode(s[2])
  
        if dirLon == "W":
          lon= (-GPS.decode(s[4]))
        else:
          lon= GPS.decode(s[4])
  
      elif data[0:6] == "$GPVTG" and date_flag==1 :
        s = data.split(",")
        true_north_dev = float(s[1]) # deviation from true north of the earth
        velocity = float(s[7]) * 1000 / 3600 #velocity in m/s
  
      return (status,lat,lon,alt_ellipsoid,true_north_dev,velocity)
  @staticmethod
  def decode(coord):
      # DDDMM.MMMMM -> DD.DDDDDDD deg
      v = coord.split(".")
      head = v[0]
      tail =  v[1]
      deg = head[0:-2]
      min = head[-2:]
      min_= float(min+"."+tail)
      deg_= round(float(deg) + min_/60,7) 
      return deg_
  
  @staticmethod
  def gps_data_handler():
    global date_flag, pub1, pub2
    pub1 = rospy.Publisher('/OB_gps/general_info', gps_info, queue_size=10)
    pub2 = rospy.Publisher('/OB_gps/velocity_info', gps_velocity, queue_size=10)
  
    rospy.init_node('OB_gps', anonymous=True)
    date_flag=0
    rate = rospy.Rate(10) # 10hz
    while (1):
      data = GPS_serial.readline()
      GPS.parseGPSparseGPS(data)
      rate.sleep()
  
if __name__ == '__main__':
  global GPS_serial

  port = "/dev/serial0"
  GPS_serial = serial.Serial(port, baudrate = 9600, timeout = 0.5)
  GPS.gps_data_handler()
  rospy.spin()
