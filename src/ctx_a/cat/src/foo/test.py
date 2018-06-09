#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('foo')
import sys
import rospy
import numpy as np
import cv2
import os
import subprocess
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class image_converter:
	def __init__(self):
		rospy.init_node('test', anonymous=True)

		self.bridge = CvBridge()
		self.image_pub_1 = rospy.Publisher("/camera/ctx_a/segmentation_output",Image, queue_size=10)       #Segmented stream
		self.image_pub_2 = rospy.Publisher("/camera/ctx_a/segmentation_output_depth",Image, queue_size=10) #Depth stream
		self.image_pub_3 = rospy.Publisher("/camera/ctx_a/line_output",Image, queue_size=10)               #Line boundary stream
		self.image = None
		self.image_sub_1 = rospy.Subscriber("/stereo_camera/left/image_rect_color",Image,self.callback)    #RGB image stream
		self.image_sub_2 = rospy.Subscriber("/stereo_camera/depth/depth_registered",Image,self.callback1)  #Depth stream

	def callback(self,data1):
		try:
			global d
			cv_image = self.bridge.imgmsg_to_cv2(data1,"bgr8")
			os.chdir("output")
			filename ="image{:04d}.jpg".format(d)
			cv2.imwrite(filename, cv_image)
			d += 1

		except CvBridgeError as e:
			print (e)

	def callback1(self,data2):
                try:
                        global d1
                        cv_image = self.bridge.imgmsg_to_cv2(data2,"32FC1")
                        os.chdir("output_depth")
                        filename ="image{:04d}.jpg".format(d1)
                        cv2.imwrite(filename, cv_image)
                        d1 += 1

                except CvBridgeError as e:
                        print (e)


	def publish_image(self):
		try:
			path="seg_output"
			os.chdir(path)
			l=os.listdir(path)
			l=sorted(l)
			filename=l[-2]
			cv_image=cv2.imread(filename)
			self.image_pub_1.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
			
			path="output_depth"
			os.chdir(path)
			l=os.listdir(path)
			l=sorted(l)
			filename=l[-1]
			cv_image=cv2.imread(filename)
			self.image_pub_2.publish(self.bridge.cv2_to_imgmsg(cv_image, "32FC1"))

			path="line_output"
			os.chdir(path)
			l=os.listdir(path)
			l=sorted(l)
			filename=l[-1]
			cv_image=cv2.imread(filename)
			self.image_pub_3.publish(self.bridge.cv2_to_imgmsg(cv_image, "32FC1"))


		except:
			pass


if __name__ == '__main__':
	d = 0
	d1 = 0
	ic = image_converter()
	r = rospy.Rate(5000)
  	while not rospy.is_shutdown():
		ic.publish_image()
		r.sleep()

