#include <imu_processor.h>



int main(int argc, char **argv){

    ros::init(argc, argv, "imu_processor");
    ros::NodeHandle nh("~");
    ROS_INFO("IMU processor start...");
    
    my_imu::IMU_Processor imuProcessor(nh);
    
    while(ros::ok()){
        ros::spinOnce();
    }
}

