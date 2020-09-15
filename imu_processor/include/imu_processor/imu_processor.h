
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Transform.h>
#include <vector>
#include <eigen3/Eigen/Core>

using namespace std;

namespace my_imu{

class IMU_Processor{
public:
    IMU_Processor(ros::NodeHandle nh);
    ~IMU_Processor(){};

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    void IMU_Callback(sensor_msgs::Imu imu);
    bool is_init_;
    vector<sensor_msgs::Imu> measures_;
    sensor_msgs::Imu imu_bias_;
    Eigen::Vector3d g_bias_;
    double delta_t_;
    long long accum_g_x, accum_g_y, accum_g_z;
    Eigen::Vector3d velocity_w_;
    Eigen::Matrix3d R_w_i_, R_i_w_;     // init transform from IMU to World.
    Eigen::Vector3d T_;                 // position in world frame.
};


}
