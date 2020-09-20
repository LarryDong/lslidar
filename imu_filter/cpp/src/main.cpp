

#include "KVH1750.h"
#include "IMU_filter.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;


int g_counter = 0;
int main(){   

    static size_t cnt = 0;
    my_imu::imu_1750 imu("/dev/ttyUSB0");
    my_imu::Imu_Data data;
    my_filter::MadgwickFilter filter(1000);     // MadgwickFilter (much) better than Mahony.
    

    Eigen::Vector3d acc_init;
    bool is_init = false;
    const int init_time_counter = 2000;     // 1s.

    // calculate bias rotation;
    cout << "Calculating bais..." << endl;
    for(int i=0; i<init_time_counter; ++i){
        imu.get_data(data);
        Eigen::Vector3d acc(data.acc[0], data.acc[1], data.acc[2]);
        acc_init += acc;
    }
    acc_init.normalize();       // use norm instead of counter to avoid >1;
    // cout << "Acc init: " << acc_init << endl;
    
    // calculate eular angle.
    Eigen::Vector3d ea_w_i0;
    // acc = R_i0_w(a_real - [0,0,-g]) + noise; |-> (R_i0_w)*[0,0,g] = acc_init
    bool is_large_pitch = false; // depend ZYX or ZXY eular angle.
    if (!is_large_pitch)
        ea_w_i0 << atan2(-acc_init[1], acc_init[2]), asin(acc_init[0]), 0;  // R*g = [sin(y), -cos(y)*sin(x), cos(x)*cos(y)]
    else
        ea_w_i0 << -asin(acc_init[1]), atan2(acc_init[0], acc_init[2]), 0;  // R*g = [cx*sy, -sx, cx*cy]

    Eigen::Matrix3d R_i0_w, R_w_i0;
    // Attention! Rotation from world2imu is ZYX from imu2world !!!
    R_i0_w = Eigen::AngleAxisd(ea_w_i0[0], Eigen::Vector3d::UnitX()) 
            * Eigen::AngleAxisd(ea_w_i0[1], Eigen::Vector3d::UnitY()) 
            * Eigen::AngleAxisd(ea_w_i0[2], Eigen::Vector3d::UnitZ());
    R_w_i0 = R_i0_w.transpose();
    
    // filter.setInitRotationFromIMU0ToWorld(R_w_i0);
    filter.R_w_i0_ = R_w_i0;
    cout << "Init IMU 2 world Eular angle: " << endl;
    cout << fixed << setprecision(4) << ea_w_i0[0] << ", " << ea_w_i0[1] << ", " << ea_w_i0[2] << "]" << endl;

    // calculate
    cout << "Begin to calcualte..." << endl;
    Eigen::Vector3d v_w(0, 0, 0), position(0, 0, 0);

    const double delta_t = 1.0f / 1000;
    while (true){
        imu.get_data(data);
        if (data.valid != 0x77)
            continue;

        Eigen::Vector3d acc(data.acc[0], data.acc[1], data.acc[2]);
        acc *= Gravity;
        Eigen::Vector3d gyro(data.gyro[0], data.gyro[1], data.gyro[2]);
        
        filter.updateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);

        // calculate position
        filter.R_i0_i_ = filter.calcRotationMatrix(filter.q0_, filter.q1_, filter.q2_, filter.q3_);
        // a_m = R_i_w (a_real - g); R_i_w = R_i_i0 * R_i0_w
        Eigen::Matrix3d R_i_w = filter.R_w_i0_ * filter.R_i0_i_;

        // Eigen::Vector3d acc_w = R_w_i0 * acc + Eigen::Vector3d(0, 0, -Gravity);
        Eigen::Vector3d acc_w = R_w_i0 * acc + Eigen::Vector3d(0, 0, -Gravity);

        // Eigen::Vector3d delta_position = v_w * delta_t + 0.5 * acc_w * delta_t * delta_t;
        Eigen::Vector3d delta_position = v_w * delta_t;

        v_w += acc_w * delta_t;
        position += delta_position;

        g_counter++;
        if (g_counter > 100){
            g_counter = 0;
            // filter.printEularAngle();
            cout << "V_w: "<<v_w<<", Delta position: " << delta_position << endl;
            cout << "Position: [" << fixed << setprecision(4) << position[0] << ", " << position[1] << ", " << position[2] << "]." << endl;
            // cout << "acc_w: [" << fixed << setprecision(4) << acc_w[0] << ", " << acc_w[1] << ", " << acc_w[2] << "]." << endl;R_w_i0
        }
    }
    return 0;
}



