
#ifndef IMU_FILTER_H__
#define IMU_FILTER_H__

#include <vector>
#include <iostream>
#include <iomanip>

namespace my_filter{

using namespace std;

class BasicFilter{
public:
    BasicFilter() = default;
    BasicFilter(float frequency) : frequency_(frequency), q0_(1), q1_(0), q2_(0), q3_(0) { }
    void quaternion2RotationMatrix(void);
    void printEularAngle(void);
    void printQuaternion(void);
    virtual void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) = 0;
    float frequency_;
    float q0_, q1_, q2_, q3_;
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

