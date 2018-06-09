VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator [1]

Takes in Visual and Odometry data from a calibrated pinhole camera (Raspberry Camera) and a 9-DOF Inertial Measurement Unit (BNO055)

Methodology:

Open two separate terminals, launch the vins_estimator, and rviz repectively by issuing the following commands:


roslaunch vins_estimator slam_a.launch
roslaunch vins_estimator vins_rviz.launch

We can even run without having the extrinsic parameters between camera and IMU. We will calibrate them online. Replace the first command with:
roslaunch vins_estimator slam_a_no_extrinsic_param.launch

Topics of interest: 

Image topic: /raspicam_node/image_indoor_blimp/compressed

IMU topic: /imu_bosch/data


Topics being published:

/clock
/feature_tracker/feature
/feature_tracker/feature_img
/feature_tracker/restart
/pose_graph/base_path
/pose_graph/camera_pose_visual
/pose_graph/key_odometrys
/pose_graph/match_image
/pose_graph/match_points
/pose_graph/no_loop_path
/pose_graph/pose_graph
/pose_graph/pose_graph_path
/vins_estimator/camera_pose
/vins_estimator/camera_pose_visual
/vins_estimator/extrinsic
/vins_estimator/history_cloud
/vins_estimator/imu_propagate
/vins_estimator/key_poses
/vins_estimator/keyframe_point
/vins_estimator/keyframe_pose
/vins_estimator/odometry
/vins_estimator/path
/vins_estimator/point_cloud
/vins_estimator/relo_relative_pose
/vins_estimator/relocalization_path



References:

[1] VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen arXiv:1708.03852
    Link: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
    Paper: https://arxiv.org/abs/1708.03852v1
