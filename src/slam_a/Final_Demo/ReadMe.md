VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator [1]

Takes in Visual and Odometry data from a calibrated pinhole camera (Raspberry Camera) and a 9-DOF Inertial Measurement Unit (BNO055)

Methodology:

Open two separate terminals, launch the vins_estimator, and rviz repectively by issuing the following commands:


roslaunch vins_estimator slam_a.launch
roslaunch vins_estimator vins_rviz.launch

We can even run without having the extrinsic parameters between camera and IMU. We will calibrate them online. Replace the first command with:
roslaunch vins_estimator slam_a_no_extrinsic_param.launch

Topics of interest: 

Indoor Blimp: 

Topic Name: /image_indoor_blimp/compressed/ 
Description: This is a topic used for sending Compressed image from pi camera (RGB, resolution of 410 x 308)

Topic Name: /indoor_imu/data/
Description: This is a topic which publishes the orientation (qaternion) and acceleration of the IMU

Outdoor Blimp:

Topic Name: /Outdoor_Blimp/cam_imu/raw
Description: This is a topic used for sending Compressed image from pi camera (RGB, resolution of 410 x 308)

Topic Name: /Outdoor_Blimp/temporary
Description: This is a topic which publishes the orientation (qaternion) and acceleration of the IMU

Topics being published:

Topic Name: /vins_estimator/point_cloud
Description: Visualisation of Point Cloud at any given instance

Topic Name: /vins_estimator/history_cloud 
Description: Visualisation of Point Cloud with all points visualised

Topic Name: /feature_tracker/feature_img 
Description: Image with tracked features

Topic Name: /vins_estimator/path 
Description: Localization path

Topic Name: /vins_estimator/camera_pose_visual 
Description: Camera Orientation



References:

[1] VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen arXiv:1708.03852
    Link: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
    Paper: https://arxiv.org/abs/1708.03852v1
