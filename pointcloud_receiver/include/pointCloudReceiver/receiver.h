#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

namespace my_receiver{

class PointcloudReceiver{
public:
    PointcloudReceiver(ros::NodeHandle node){
        ROS_INFO("Construct the receiver");
        pc_sub_ = node.subscribe("/lslidar_point_cloud", 10, &PointcloudReceiver::pointcloudCallback, (PointcloudReceiver*)this);
    };
    ~PointcloudReceiver(){};

private:
    ros::Subscriber pc_sub_;
    void pointcloudCallback(sensor_msgs::PointCloud2 data){
        ROS_INFO("Call back...");
    }
};


};