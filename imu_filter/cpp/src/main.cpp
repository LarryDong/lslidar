

#include "KVH1750.h"
// #include "IMU.h"
// #include "MadgwickAHRS.h"
#include "IMU_filter.h"

#include <iostream>
using namespace std;


int g_counter = 0;
int main(){   

    static size_t cnt = 0;
    my_imu::imu_1750 imu("/dev/ttyUSB0");
    my_imu::Imu_Data data;
    my_filter::MadgwickFilter filter(1000);     // MadgwickFilter (much) better than Mahony.
    
    while (true){
        imu.get_data(data);
        float z_angle = 0.0f;
        if (data.valid == 0x77){
            g_counter++;        // output speed control.
            filter.updateIMU(data.gyro[0], data.gyro[1], data.gyro[2], data.acc[0] * Gravity, data.acc[1] * Gravity, data.acc[2] * Gravity);
            if(g_counter>100){
                g_counter = 0;
                filter.printEularAngle();
            }
        }
    }
    return 0;
}


