# lslidar

codes for Lidar+IMU system.

When compiling Lidar, need pcap.h
1. sudo apt install libpcap-dev
if error: "libpcap-dev depends libpcap0.8-dev but it is not going to be installed", remove 0.8 version by: 
2. sudo apt-get remove libpcap0.8

When running:
1. sudo chmod +777 /dev/ttyUSB0
2. roslaunch lslidar/my_launch/driver.launch
3. roslaunch lslidar/my_launch/decoder.launch
4. rosrun imu_driver imu_driver_node
5. launch rviz, set "laser_link" and add "pointCloud2"
6. rosbag record -a     # record all data

Attention:
1. Rotation speed: +- 490 degree/s. Larger than this value, no output so that filter failed.
