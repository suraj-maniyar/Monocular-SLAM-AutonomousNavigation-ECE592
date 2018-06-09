#!/usr/bin/env python
import rospy

from OB_gps.msg import gps_info
from OB_gps.msg import gps_velocity
import serial

global dmy_string, date_flag

def parseGPS(data):
    global dmy_string, date_flag, pub1, pub2

    if data[0:6] == "$GPGGA":
      s = data.split(",")
      if s[6] == "0" or date_flag!=1 :
          print "no satellite data available"
          return
      time = s[1][0:2] + ":" + s[1][2:4] + ":" + s[1][4:6]
      dirLat = s[3]
      dirLon = s[5]
      alt = s[9] + " m"
      sat = s[7]
      
      alt_ellipsoid = float(s[9])
      status = int(s[6])
      if dirLat == "S":
        lat= (-decode(s[2])) 
      else:
        lat= decode(s[2])

      if dirLon == "W":
        lon= (-decode(s[4]))
      else:
        lon= decode(s[4])
      #print "Time(UTC): %s-- Latitude: %s(%s)-- Longitude:%s(%s)-- Altitute:%s--(%s satellites)" %(time, lat, dirLat, lon, dirLon, alt, sat) 
      time_stamp = time + " " + dmy_string
      pub1.publish(status,time_stamp,lat,lon,alt_ellipsoid)

    elif data[0:6] == "$GPVTG" and date_flag==1 :
      s = data.split(",")
      true_north_dev = float(s[1]) # deviation from true north of the earth
      velocity = float(s[7]) * 1000 / 3600 #velocity in m/s

      pub2.publish(true_north_dev,velocity)

    elif data[0:6] == "$GPRMC":
      s = data.split(",")
      if int(s[9][4:6])<=30:
        date = s[9][0:2]
        month = s[9][2:4]
        year = s[9][4:6]

        dmy_string= date + "-" + month + "-" + year
        date_flag=1

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


def gps_data_handler():
  global date_flag, pub1, pub2
  pub1 = rospy.Publisher('/OB_gps/general_info', gps_info, queue_size=10)
  pub2 = rospy.Publisher('/OB_gps/velocity_info', gps_velocity, queue_size=10)

  rospy.init_node('OB_gps', anonymous=True)
  date_flag=0
  rate = rospy.Rate(10) # 10hz
  while (1):
    data = GPS_serial.readline()
    parseGPS(data)
    rate.sleep()

if __name__ == '__main__':
  global GPS_serial

  port = "/dev/serial0"
  GPS_serial = serial.Serial(port, baudrate = 9600, timeout = 0.5)
  gps_data_handler()
  rospy.spin()
