#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <string>

namespace my_receiver{

class PointcloudReceiver{
public:
    PointcloudReceiver(ros::NodeHandle node);
    ~PointcloudReceiver(){};

private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Publisher pc_pub_;
    void pointcloudCallback(sensor_msgs::PointCloud2 data);
};

};