CTX- B

Set up Nvidia Digits on any Host Computer and on Jetson TX1 using the link:
https://github.com/dusty-nv/jetson-inference

We have used ‘Image Segmentation with SegNet’. Follow the steps given in the link above to set up. The model given in the above link  trains the model on Aerial Drone Dataset. We have instead used our own dataset from an indoor construction site. 
1. The dataset collected is first annotated using the link: https://supervise.ly/
2. We download the annotated json files which can be converted to labelled images.
We can now follow the steps give in the github link to train on our dataset collected. 

After training the model on our dataset the model is then downloaded and transferred to jetson for inference on testing images. (Make sure that the jeston is set up correctly according to the github link). 

We now set our ROS workspace. The ROS workspace has to be created inside the folder /jetson-inference/build/aarch64/bin. To create this, follow the link: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
 
Copy the file ‘pub_sub.py’ to the workspace created (inside the src folder).
 
When the topic  ‘/raspicam_node/image/compressed’  is subscribed to, we start receiving the live images from camera which is used in semantic segmentation of that image. 

The input camera image (input_camera.png) and final output (final_output.png) that are obtained after segmentation are saved inside the package ‘beginner_tutorials/src/’. 

These images are also published back through topic ‘/camera/image_1’ (contains segmented image) and ‘/camera/image_2’ (contains original images being captured from camera).

Package:
1. Beginner_tutorials                                                                  

Ros Node: 
1. image_listener

Ros Topic: 
Subscribing to:
 1. /raspicam_node/image/compressed          
Publishing :
 1. /camera/image_1
 2. /camera/image_2


In the terminal, run the following file:
rosrun beginner_tutorials pub_sub.py
