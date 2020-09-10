


#include <iostream>
#include "KVH1750.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu toRosMsg(const my_imu::Imu_Data & rawdata){
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "KVH1750";
    imu_data.header.seq = rawdata.sequence_num;
    imu_data.angular_velocity.x = rawdata.acc[0];
    imu_data.angular_velocity.y = rawdata.acc[1];
    imu_data.angular_velocity.z = rawdata.acc[2];
    imu_data.linear_acceleration.x = rawdata.gyro[0];
    imu_data.linear_acceleration.y = rawdata.gyro[1];
    imu_data.linear_acceleration.z = rawdata.gyro[2];
}


int main(int argc, char **argv){

    ros::init(argc, argv, "imu_driver");
    ros::NodeHandle nh("~");
    ROS_INFO("IMU serial communication start...");

    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
    my_imu::imu_1750 imu("/dev/ttyUSB0");   // open ttyUSB0. Make sure to 'sudo chmod 777 /dev/ttyUSB0'

    while(ros::ok()){
        my_imu::Imu_Data raw_data;
        sensor_msgs::Imu imu_data;
        if(imu.get_data(raw_data)){
            imu_data = toRosMsg(raw_data);
            pub.publish(imu_data);
            ros::spinOnce();
        }
        else{
            ROS_WARN("Cannot get IMU data...");
        }
    }

    return 0;
}
