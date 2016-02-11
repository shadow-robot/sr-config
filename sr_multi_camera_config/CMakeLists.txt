cmake_minimum_required(VERSION 2.8.3)
project(sr_multi_camera_config)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  industrial_extrinsic_cal
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
)
