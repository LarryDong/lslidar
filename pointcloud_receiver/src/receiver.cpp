
#include "receiver.h"

namespace my_receiver{

PointcloudReceiver::PointcloudReceiver(ros::NodeHandle node){
    ROS_INFO("Construct the receiver");
    nh_ = node;
    pc_sub_ = nh_.subscribe("/lslidar_point_cloud", 10, &PointcloudReceiver::pointcloudCallback, (PointcloudReceiver *)this);
}

void PointcloudReceiver::pointcloudCallback(sensor_msgs::PointCloud2 pc2){

    // ROS_INFO("pointcloud Callback...");
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);

    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/PointCloud", 10);
    // ROS_INFO_STREAM("Channel: " << pc.channels.size() << ", points: " << pc.points.size());
    ROS_INFO_STREAM("Channel 0: " << pc.channels[0].name << ", 1: " << pc.channels[1].name << ", 2: " << pc.channels[2].name);
    
    pc_pub_.publish(pc);
}

}


