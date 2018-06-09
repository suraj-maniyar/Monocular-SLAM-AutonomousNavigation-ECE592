#!/usr/bin/env python

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import * #CompressedImage, Imu, MagneticField, Temperature, *
import rospy

imu_topic = ""
image_topic = ""
print("Enter o/i")

blimp_mode = raw_input()

if blimp_mode == "o":
    print("Syncing OutdoorBlimp topics")

    imu_topic="/Outdoor_Blimp/cam_imu/raw"
    #image_topic="/Outdoor_Blimp/image_raw/compressed"
    image_topic="/Outdoor_Blimp/temporary/compressed"
    #image_topic="/Outdoor_Blimp/temporary/compressed"
    pub_image_topic = "/Outdoor_Blimp/image_sync/compressed"

elif blimp_mode == "i":
    print("Syncing IndoorBlimp topics")

    imu_topic="/imu/raw"
    image_topic="/raspicam_node/image/compressed"
    pub_image_topic = "/raspicam_node/image_sync/compressed"


pub_image = rospy.Publisher(pub_image_topic, CompressedImage, queue_size=1)
pub_imu = rospy.Publisher(imu_topic+"_sync", Imu, queue_size=1)

def gotimage(image, imu):
    print("Inside callback")
    #print "got an Image and imu SYNC"
    pub_image.publish(image)

    imu_msg = Imu()
    imu_msg.header.stamp = imu.header.stamp
    imu_msg.header.frame_id = imu.header.frame_id
    imu_msg.header.seq = imu.header.seq
    imu_msg.linear_acceleration.x = imu.linear_acceleration.x
    imu_msg.linear_acceleration.y = imu.linear_acceleration.y
    imu_msg.linear_acceleration.z = imu.linear_acceleration.z
    imu_msg.angular_velocity.x = imu.angular_velocity.x
    imu_msg.angular_velocity.y = imu.angular_velocity.y
    imu_msg.angular_velocity.z = imu.angular_velocity.z
    imu_msg.orientation.x = imu.orientation.x
    imu_msg.orientation.y = imu.orientation.y
    imu_msg.orientation.z = imu.orientation.z
    imu_msg.orientation.w = imu.orientation.w

    pub_imu.publish(imu_msg)

rospy.init_node('sync')
#image_info_sub = Subscriber("/raspicam_node/camera_info", CameraInfo) #CompressedImage)
image_sub = Subscriber(image_topic, CompressedImage)

imu_sub = Subscriber(imu_topic, Imu)
#print(imu_sub.header)
#print(image_sub.header)

ats = ApproximateTimeSynchronizer([image_sub, imu_sub], queue_size=1, slop=0.55, allow_headerless=False)
ats.registerCallback(gotimage)

rospy.spin()

