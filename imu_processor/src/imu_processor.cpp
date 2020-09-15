
#include <imu_processor.h>


using namespace std;


namespace  my_imu{


IMU_Processor::IMU_Processor(ros::NodeHandle nh):
    nh_(nh),
    delta_t_(0.001),
    is_init_(false),
    g_bias_(Eigen::Vector3d(0, 0, 9.8)),
    R_w_i_(Eigen::Matrix3d::Identity(3,3)),
    velocity_w_(Eigen::Vector3d(0, 0, 0)),
    accum_g_x(0.0), accum_g_y(0.0), accum_g_z(0.0)
{
    measures_.resize(0);
    ROS_INFO("Construct the IMU Processor.");
    ROS_INFO("Calculating G bias...");
    sub_ = nh_.subscribe("/imu_driver/imu", 10, &IMU_Processor::IMU_Callback, (IMU_Processor *)this);
    pub_ = nh_.advertise<geometry_msgs::Transform>("transform", 10);
}


void IMU_Processor::IMU_Callback(sensor_msgs::Imu imu){
    // ROS_INFO("IMU procecessor callback...");
    if(!is_init_){
        measures_.emplace_back(imu);
        // initialize to calcualte G bias. For 5 second.
        const double init_time = 10.0;       // 5s for init.
        if(measures_.size() >= static_cast<long long>(init_time/delta_t_)){
            for(const sensor_msgs::Imu &m : measures_){
                // imu_bias_.angular_velocity.x += m.angular_velocity.x;
                // imu_bias_.angular_velocity.y += m.angular_velocity.y;
                // imu_bias_.angular_velocity.z += m.angular_velocity.z;
                // imu_bias_.linear_acceleration.x += m.linear_acceleration.x;
                // imu_bias_.linear_acceleration.y += m.linear_acceleration.y;
                // imu_bias_.linear_acceleration.z += m.linear_acceleration.z;

                accum_g_x += m.linear_acceleration.x;
                accum_g_y += m.linear_acceleration.y;
                accum_g_z += m.linear_acceleration.z;
            }
            // imu_bias_.angular_velocity.x /= measures_.size();
            // imu_bias_.angular_velocity.y /= measures_.size();
            // imu_bias_.angular_velocity.z /= measures_.size();
            // imu_bias_.linear_acceleration.x /= measures_.size();
            // imu_bias_.linear_acceleration.y /= measures_.size();
            // imu_bias_.linear_acceleration.z /= measures_.size();
            
            // ROS_INFO_STREAM("--> Acc:( " << imu_bias_.linear_acceleration.x << ", " << imu_bias_.linear_acceleration.y << ", " << imu_bias_.linear_acceleration.z << "). ");
            // ROS_INFO_STREAM("--> Gyro:( " << imu_bias_.angular_velocity.x << ", " << imu_bias_.angular_velocity.y << ", " << imu_bias_.angular_velocity.z << "). ");
            g_bias_[0] = accum_g_x/measures_.size();
            g_bias_[1] = accum_g_y/measures_.size();
            g_bias_[2] = accum_g_z/measures_.size();
            ROS_INFO("G Bias: -------------");
            ROS_INFO_STREAM("[ " << g_bias_[0] << ", " << g_bias_[1] << ", " << g_bias_[2] << "]. ");
            is_init_ = true;
        }
        return ;
    }

    // after initialization, begin to calculate distance change.
    // 1. calculate position. x = x_0 + v*t + 1/2 * a * t^2;
    // ROS_INFO_STREAM("R_i_w: " << R_i_w_);
    // ROS_INFO_STREAM("delta_distance: " << delta_distance[0] << ", " << delta_distance[1] << ", " << delta_distance[2]);

    R_i_w_ = R_w_i_.transpose();
    Eigen::Vector3d velocity_i = R_i_w_ * velocity_w_;
    Eigen::Vector3d a(Eigen::Vector3d(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z));
    a = a - g_bias_;

    T_ += a;
    ROS_INFO_STREAM("Accumulated a: [" << T_[0] << ", " << T_[1] << ", " << T_[2] << "]. ");


    // Eigen::Vector3d delta_distance = velocity_i * delta_t_ + 0.5 * a * delta_t_ * delta_t_;
    // Eigen::Vector3d delta_v = a * delta_t_;

    
    // velocity_w_ += R_w_i_ * delta_v;
    // T_ += R_w_i_ * delta_distance;
    // ROS_INFO_STREAM("Position: [" << T_[0] << ", " << T_[1] << ", " << T_[2] << "]. ");

    // // 2. calculate rotation.

    // double ax = imu.angular_velocity.x * delta_t_;
    // double ay = imu.angular_velocity.y * delta_t_;
    // double az = imu.angular_velocity.z * delta_t_;
    // Eigen::Matrix3d rx, ry, rz;
    // rx << 1, 0, 0, 0, cos(ax), -sin(ax), 0, sin(ax), cos(ax);
    // ry << cos(ay), 0, sin(ay), 0, 1, 0, -sin(ay), 0, cos(ay);
    // rz << cos(az), -sin(az), 0, sin(az), cos(az), 0, 0, 0, 1;
    // Eigen::Matrix3d delta_R =  
}   

}

