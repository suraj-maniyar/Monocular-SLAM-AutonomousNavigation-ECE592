SLAM Team A - ORB SLAM/ORB SLAM2
-------------------------------------------------------------------------------------------------------------
ROS for monocular [1]

The objective of implementing SLAM for Monocular camera is for efficient localization and tracking of both indoor and outdoor blimps.

ORB-SLAM1 can be downloaded from https://github.com/raulmur/ORB_SLAM

Make sure you build all the required dependencies as mentioned on the github page.
We have updated the code for subscribing from compressed Images for faster data transmission as compared to raw image.

-------------------------------------------------------------------------------------------------------------
ROS for RGB-D using ZED camera [2]

The objective of implementing SLAM for RGB-D using ZED camera is for efficient localization and tracking husky.

ORB-SLAM2 can be downloaded from https://github.com/raulmur/ORB_SLAM2

Make sure you build all the required dependencies as mentioned on the github page.
We have updated the code as per our requirements for publishing point cloud.

-------------------------------------------------------------------------------------------------------------
1. Zed ROS Wrapper
This is a wrapper for the Zed camera which publishes images from a stereo camera. Use 'roslaunch zed_wrapper zed.launch’ to run the zed node.
It publishes many topics out of which we choose the following:-
* /zed/rgb/image_raw_color
* /zed/depth_registered/depth


2. ORB-SLAM - 1
Monocular 
This code uses rgb images from monocular camera and calculates the Essential Matrix which can be visualized as a 3D map in RViz.


It subscribes from these topics:-
* /raspicam_node/image_indoor_blimp/compressed
* /raspicam_node/image_outdoor/compressed


It publishes these topics:-
* ORB_SLAM/World
* ORB_SLAM/Camera


3. ORB-SLAM - 2
This code runs on both Monocular and RGB-D data.


It subscribes from these topics:-
* /zed/left/image_raw_color OR /zed/right/image_raw_color


RGB-D
It subscribes from these topics:-
* /zed/rgb/image_raw_color
* /zed/depth/depth_registered


It publishes these topics:-
* /slam/PointCloud
* /slam/pos

-------------------------------------------------------------------------------------------------------------
Camera Calibration :

** ZED Camera: ZED Camera calibration file can be downloaded from the link:  http://calib.stereolabs.com/?SN=1010 where one needs to replace “1010” with the Serial Number of the ZED Camera. 
Alternatively, ZED can be calibrated from the Calibration Run File provided with ZED SDK at the path ‘/usr/local/zed/tools/ ‘

** Monocular Camera - Raspberry Pi Camera : OpenCV camera_calibration.cpp file is used for calibration of Raspberry Pi Camera. A 9 x 6 chessboard 90 second video  with frame rate of 1 fps  having 256 * 256 resolution captured from the Raspberry Pi Camera is used.  Various parameters which are changed in XML file include the size of the box which is 24 mm, Number of frames to be used for calibration is 100, and setting flag true for tangential distortion coefficients and change in principal point during global optimization. It is to be noted that file has an error in loading video feed and change has to be made in readStringList function with a try-catch function. 

-------------------------------------------------------------------------------------------------------------
References:

1. Mur-Artal, Raul, Jose Maria Martinez Montiel, and Juan D. Tardos. "ORB-SLAM: a versatile and accurate monocular SLAM system." IEEE Transactions on Robotics 31.5 (2015): 1147-1163.
2. Mur-Artal, Raul, and Juan D. Tardós. "Orb-slam2: An open-source slam system for monocular, stereo, and rgb-d cameras." IEEE Transactions on Robotics 33.5 (2017): 1255-1262.
