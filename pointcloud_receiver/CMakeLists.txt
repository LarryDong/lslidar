cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_receiver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  dynamic_reconfigure
  lslidar_c32_driver
  lslidar_c32_msgs
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES pointCloudReceiver
 CATKIN_DEPENDS 
 angles 
 dynamic_reconfigure 
 lslidar_c32_driver 
 lslidar_c32_msgs 
 pcl_conversions 
 pcl_ros 
 pluginlib 
 roscpp 
 sensor_msgs 
 std_msgs 
 tf
)

include_directories(
    include/pointCloudReceiver
    ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_receiver_node src/receiver_node.cpp src/receiver.cpp)
target_link_libraries(lidar_receiver_node
    ${catkin_LIBRARIES}
)
