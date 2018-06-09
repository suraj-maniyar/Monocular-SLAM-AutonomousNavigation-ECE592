#include <darknet_ros/YoloObjectDetector.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");
  darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);

  ros::spin();
  return 0;
}