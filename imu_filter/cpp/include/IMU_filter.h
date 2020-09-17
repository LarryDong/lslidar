
#ifndef IMU_FILTER_H__
#define IMU_FILTER_H__

#include <vector>
#include <iostream>
#include <iomanip>
#include <eigen3/Eigen/Core>

namespace my_filter{

using namespace std;

class BasicFilter{
public:
    BasicFilter() = default;
    BasicFilter(float frequency) : frequency_(frequency),
                                    q0_(1), q1_(0), q2_(0), q3_(0),
                                    R_w_i0_(Eigen::Matrix3d::Identity(3, 3)) {}

    Eigen::Matrix3d calcRotationMatrix(float q0, float q1, float q2, float q3);
    Eigen::Vector3d calcEularAngelZYX(float q0, float q1, float q2, float q3);
    Eigen::Vector3d calcEularAngelZYX(Eigen::Matrix3d R);

    void printEularAngle(void);
    void printQuaternion(void);
    void printRotationMatrix(void);

    virtual void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) = 0;
    float frequency_;
    float q0_, q1_, q2_, q3_;           // quarterion from IMU_t to IMU_0.
    
    Eigen::Vector3d eularAngle_;        // ZYX Eular angle.

    void setInitRotationFromIMU0ToWorld(Eigen::Matrix3d R) { R_w_i0_ = R; }

    Eigen::Matrix3d R_i0_i_;                 // R_ from quarterion to get R
    Eigen::Matrix3d R_w_i0_;        // a_measure = R_i0_w_(a_w - g) + noise;
};



class MahonyFilter : public BasicFilter{

public:
    MahonyFilter(float frequency, float kp = 0.5f, float ki = 0.0f)
        : BasicFilter(frequency), twoKp_(2 * kp), twoKi_(2 * ki), integralFBx_(0), integralFBy_(0), integralFBz_(0) {;}
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) override;

private:
    float twoKp_, twoKi_;
    float integralFBx_, integralFBy_, integralFBz_; // integral error terms scaled by Ki
};


class MadgwickFilter : public BasicFilter{

public:
    MadgwickFilter(float frequency, float beta = 0.1f)
        : BasicFilter(frequency), beta_(beta) {}
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) override;

private:
    float beta_;
};

}

#endif

