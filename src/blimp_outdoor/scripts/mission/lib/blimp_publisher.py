#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  2 16:10:20 2018

@author: vijayaganesh
"""
import BNO055
#import gps.GPS as GPS
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Pose, TwistStamped, Vector3
from outdoor_blimp.msg import gps_info, gps_velocity
from std_msgs.msg import Header
import rospy
import serial
import time

class Blimp_Publisher:
  """
  Principle Class of Blimp
  """
  def __init__(self):
    self.pose_publisher = rospy.Publisher('/Outdoor_Blimp/imu/pose',PoseStamped,queue_size = 10)
    self.rpy_publisher = rospy.Publisher('/Outdoor_Blimp/imu/orientation',Vector3,queue_size = 10)
    self.twist_publisher = rospy.Publisher('/Outdoor_Blimp/imu/twist', TwistStamped, queue_size=10)

    # self.cam_pose_publisher = rospy.Publisher('/Outdoor_Blimp/camera/imu/pose',PoseStamped,queue_size = 10)
    # self.cam_rpy_publisher = rospy.Publisher('/Outdoor_Blimp/camera/imu/orientation',Vector3,queue_size = 10)
    # self.cam_twist_publisher = rospy.Publisher('/Outdoor_Blimp/camera/imu/twist', TwistStamped, queue_size=10)
    
    #self.gps_info_publisher =  rospy.Publisher('/Outdoor_Blimp/gps/general_info', gps_info, queue_size=10)
    #self.gps_vel_publisher = rospy.Publisher('/Outdoor_Blimp/gps/velocity_info', gps_velocity, queue_size=10)
    
    port = "/dev/serial0"
    self.gps_serial = serial.Serial(port, baudrate = 9600, timeout = 0.5)
    
    self.bno = BNO055.BNO055()
    if self.bno.begin() is not True:
        print "Error initializing device"
        exit()
        time.sleep(1)
    self.bno.setExternalCrystalUse(True)

    # self.cam_bno = BNO055.BNO055(address=0x29)
    # if self.cam_bno.begin() is not True:
    #   print "Error initializing device"
    #   exit()
    #   time.sleep(1)
    # self.cam_bno.setExternalCrystalUse(True)
    
  def publish_imu(self):
    euler_data = self.bno.getVector(BNO055.BNO055.VECTOR_EULER)
    # print('reg IMU = ' + repr(euler_data))
    # euler_data = self.bno.read_euler()
    # print('read Euler = ')
    # print(self.bno.read_euler())
    # print('getVector = ')
    # print(euler_data)
    rpy_message = Vector3(euler_data[0],euler_data[1],euler_data[2])
    # print(rpy_message)
    # print(' ')
    quaternion_data = self.bno.getQuat()
    gyro_data = self.bno.getVector(BNO055.BNO055.VECTOR_GYROSCOPE);
    linear = Vector3(0, 0, 0)
    angular = Vector3(gyro_data[0], gyro_data[1], gyro_data[2])
    header = Header()
    header.stamp = rospy.Time.now()
    quat = Quaternion()
    quat.w = quaternion_data[3]
    quat.x = quaternion_data[0]
    quat.y = quaternion_data[1]
    quat.z = quaternion_data[2]
    twist_message = TwistStamped(header, Twist(linear, angular))
    point = Point(0, 0, 0)
    pose = Pose(point, quat)
    pose_message = PoseStamped(header, pose)
    self.pose_publisher.publish(pose_message)
    self.twist_publisher.publish(twist_message)
    self.rpy_publisher.publish(rpy_message)

 #  def publish_cam_imu(self):
    # euler_data = self.cam_bno.getVector(BNO055.BNO055.VECTOR_EULER)
    # euler_data = self.bno.read_euler()
    #print('cam IMU = ' + repr(euler_data))
    # print(self.bno.read_euler())
    # print('getVector = ')
    # print(euler_data)
    # rpy_message = Vector3(euler_data[0],euler_data[1],euler_data[2])
    # print(rpy_message)
    # print(' ')
    # quaternion_data = self.cam_bno.getQuat()
    # gyro_data = self.cam_bno.getVector(BNO055.BNO055.VECTOR_GYROSCOPE);
    # linear = Vector3(0, 0, 0)
    # angular = Vector3(gyro_data[0], gyro_data[1], gyro_data[2])
    # header = Header()
    # header.stamp = rospy.Time.now()
    # quat = Quaternion()
    # quat.w = quaternion_data[3]
    # quat.x = quaternion_data[0]
    # quat.y = quaternion_data[1]
    # quat.z = quaternion_data[2]
    # twist_message = TwistStamped(header, Twist(linear, angular))
    # point = Point(0, 0, 0)
    # pose = Pose(point, quat)
    # pose_message = PoseStamped(header, pose)
    # self.cam_pose_publisher.publish(pose_message)
    # self.cam_twist_publisher.publish(twist_message)
    # self.cam_rpy_publisher.publish(rpy_message)
    
 # def publish_gps(self):
   # data = self.gps_serial.readline()
    #status,lat,lon,alt_ellipsoid,true_north_dev,velocity = GPS.parseGPS(data)
    #time_stamp = rospy.Time.now()
    
    #self.gps_info_publisher(status,time_stamp,lat,lon,alt_ellipsoid)
    #self.gps_vel_publisher(true_north_dev,velocity)

  
    
  
