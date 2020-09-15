cmake_minimum_required(VERSION 3.0.2)
project(imu_processor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
CATKIN_DEPENDS 
roscpp 
sensor_msgs
std_msgs 
)


include_directories(
    include/imu_processor
    ${catkin_INCLUDE_DIRS}
)


add_executable(imu_processor_node src/imu_processor.cpp src/imu_processor_node.cpp)
target_link_libraries(imu_processor_node 
    boost_system
    pthread
    ${catkin_LIBRARIES}
)
