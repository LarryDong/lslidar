
#include <ros/ros.h>
#include "receiver.h"



int main(int argc, char **argv){
    
    ros::init(argc, argv, "pointcloud_receiver");
    ros::NodeHandle nh("~");
    ROS_WARN("Point cloud receiver start...");

    my_receiver::PointcloudReceiver ptReceiver(nh);

    // while (ros::ok()){
    //     ros::spinOnce();
    // }
    ros::spin();

    return 0;
}
