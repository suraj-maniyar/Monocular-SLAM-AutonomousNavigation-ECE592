#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
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
    self.image_pub = rospy.Publisher("/camera/image_1",Image, queue_size=1)
    self.image_pub_2 = rospy.Publisher("/camera/image_2",Image, queue_size=1)
    
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback)

  def callback(self,data):
    print("Received an image!")
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
      cv_image = cv2.resize(cv_image, (1280, 720))
      cv2.imwrite('camera_image.png', cv_image)
      print(cv_image.shape)
      print(type(cv_image))
      print("image written ______________________________")
      
      #NET="/home/nvidia/jetson-inference/20180302-022602-9d48_epoch_1.0"
      
      #command = "./segnet-console /home/nvidia/jetson-inference/build/aarch64/bin/catkin_final/src/beginner_tutorials/src/camera_image.png /home/nvidia/jetson-inference/build/aarch64/bin/catkin_final/src/beginner_tutorials/src/final_output.png --prototxt=" + NET + "/deploy.prototxt --model=" + NET + "/snapshot_iter_4522.caffemodel --labels=" + NET + "/fpv-labels.txt --colors=" + NET + "/fpv-deploy-colors.txt --input_blob=/home/nvidia/jetson-inference/data --output_blob=score_fr"

      #print(command)
      #time.sleep(5)
      #subprocess.call(['test.sh'])
      os.system('cd /home/nvidia/jetson-inference/build/aarch64/bin && python trial.py')
      print("Segnet module has been called ______________________________")
      time.sleep(5)
      #cv2.imshow('camera_image.jpeg', cv_image)
      #cv2.waitKey(4)
      
      #crop_img = cv_image[0:300, 0:300]
      #cv2.imwrite('camera_cropped_image.jpeg', crop_img)
      
      #canny_img = cv2.Canny(cv_image,100,200)
      #cv2.imwrite('camera_image_canny.jpeg', canny_img)
#      msg = CompressedImage()
#      msg.header.stamp = rospy.Time.now()
#      msg.format = "jpeg"
#      msg.data = np.array(cv2.imencode('.jpg', canny_img)[1]).tostring()

    except CvBridgeError as e:
      print(e)
	#else:
        # Save your OpenCV2 image as a jpeg 
    #    cv2.imwrite('camera_image.jpeg', cv_image)
     #   canny_img = cv2.Canny(cv_image,100,200)
      #  cv2.imwrite('camera_image_canny.jpeg', canny_img)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.imread('/home/nvidia/jetson-inference/build/aarch64/bin/catkin_final/src/beginner_tutorials/src/final_output.png'), "bgr8"))

      self.image_pub_2.publish(self.bridge.cv2_to_imgmsg(cv2.imread('/home/nvidia/jetson-inference/build/aarch64/bin/catkin_final/src/beginner_tutorials/src/camera_image.png'), "bgr8"))
      
      print("Image Published!")
    except CvBridgeError as e:
      print(e)


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
