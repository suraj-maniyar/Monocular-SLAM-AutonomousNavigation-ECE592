#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import numpy as np

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("camera/image_2",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_1",Image,self.callback_1)
    self.image_sub_2 = rospy.Subscriber("/camera/image_2",Image,self.callback_2)

  def callback_1(self,data):
    print("Received an image!")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.imwrite('final_output.jpeg', cv_image)

    except CvBridgeError as e:
      print(e)


  def callback_2(self,data):
    print("Received an image!")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.imwrite('input.jpeg', cv_image)

    except CvBridgeError as e:
      print(e)
	#else:
        # Save your OpenCV2 image as a jpeg 
    #    cv2.imwrite('camera_image.jpeg', cv_image)
     #   canny_img = cv2.Canny(cv_image,100,200)
      #  cv2.imwrite('camera_image_canny.jpeg', canny_img)
"""
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(crop_img, "bgr8"))
    except CvBridgeError as e:
      print(e)

"""

##########################################################################

##########################################################################


def main(args):
  ic = image_converter()
  rospy.init_node('image_listener', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
