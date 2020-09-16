

#include "IMU_filter.h"
#include <iostream>
#include <math.h>

namespace my_filter{

using namespace std;
void BasicFilter::quaternion2RotationMatrix(void){
    float r11, r12, r13, r21, r22, r23, r31, r32, r33;
    float q0q0 = q0_ * q0_;
    float q0q1 = q0_ * q1_;
    float q0q2 = q0_ * q2_;
    float q0q3 = q0_ * q3_;
    float q1q1 = q1_ * q1_;
    float q1q2 = q1_ * q2_;
    float q1q3 = q1_ * q3_;
    float q2q2 = q2_ * q2_;
    float q2q3 = q2_ * q3_;
    float q3q3 = q3_ * q3_;
    r11 = q0q0 + q1q1 - q2q2 - q3q3;
    r12 = 2 * q1q2 + 2 * q0q3;
    r13 = 2 * q1q3 - 2 * q0q2;
    r21 = 2 * q1q2 - 2 * q0q3;
    r22 = q0q0 - q1q1 + q2q2 - q3q3;
    r23 = 2 * q2q3 + 2 * q0q1;
    r31 = 2 * q1q3 + 2 * q0q2;
    r32 = 2 * q2q3 - 2 * q0q1;
    r33 = q0q0 - q1q1 - q2q2 + q3q3;
}

void BasicFilter::printEularAngle(void){
float phi, theta, pasi;
    float q0q0 = q0_ * q0_;
    float q0q1 = q0_ * q1_;
    float q0q2 = q0_ * q2_;
    float q0q3 = q0_ * q3_;
    float q1q1 = q1_ * q1_;
    float q1q2 = q1_ * q2_;
    float q1q3 = q1_ * q3_;
    float q2q2 = q2_ * q2_;
    float q2q3 = q2_ * q3_;
    float q3q3 = q3_ * q3_;

    phi = atan2f(2 * (q0q1 + q2q3), 1 - 2 * (q1q1 + q2q2));
    theta = asinf(2 * (q0q2 - q1q3));
    pasi = atan2f(2 * (q0q3 + q1q2), 1 - 2 * (q2q2 + q3q3));

    cout << fixed << setprecision(5) << "Roll: " << phi << ", picth: " << theta << ", yaw: " << pasi << endl;
}

void BasicFilter::printQuaternion(void){
    cout << fixed << setprecision(4) << "Quaterion: [" << q0_ << ", " << q1_ << ", " << q2_ << ", " << q3_ << " ]" << endl;
}



// MahonyFilter
void MahonyFilter::updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1_ * q3_ - q0_ * q2_;
		halfvy = q0_ * q1_ + q2_ * q3_;
		halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi_ > 0.0f) {
			integralFBx_ += twoKi_ * halfex * (1.0f / frequency_);	// integral error scaled by Ki
			integralFBy_ += twoKi_ * halfey * (1.0f / frequency_);
			integralFBz_ += twoKi_ * halfez * (1.0f / frequency_);
			gx += integralFBx_;	// apply integral feedback
			gy += integralFBy_;
			gz += integralFBz_;
		}
		else {
			integralFBx_ = 0.0f;	// prevent integral windup
			integralFBy_ = 0.0f;
			integralFBz_ = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp_ * halfex;
		gy += twoKp_ * halfey;
		gz += twoKp_ * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / frequency_));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / frequency_));
	gz *= (0.5f * (1.0f / frequency_));
	qa = q0_;
	qb = q1_;
	qc = q2_;
	q0_ += (-qb * gx - qc * gy - q3_ * gz);
	q1_ += (qa * gx + qc * gz - q3_ * gy);
	q2_ += (qa * gy - qb * gz + q3_ * gx);
	q3_ += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
    recipNorm = 1.0f / sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= recipNorm;
	q1_ *= recipNorm;
	q2_ *= recipNorm;
	q3_ *= recipNorm;
}

// MadgwickFilter
// Madgwick filter performs (much) better than Mahony filter
void MadgwickFilter::updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
	qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
	qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
	qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0_;
		_2q1 = 2.0f * q1_;
		_2q2 = 2.0f * q2_;
		_2q3 = 2.0f * q3_;
		_4q0 = 4.0f * q0_;
		_4q1 = 4.0f * q1_;
		_4q2 = 4.0f * q2_;
		_8q1 = 8.0f * q1_;
		_8q2 = 8.0f * q2_;
		q0q0 = q0_ * q0_;
		q1q1 = q1_ * q1_;
		q2q2 = q2_ * q2_;
		q3q3 = q3_ * q3_;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;
        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta_ * s0;
		qDot2 -= beta_ * s1;
		qDot3 -= beta_ * s2;
		qDot4 -= beta_ * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0_ += qDot1 * (1.0f / frequency_);
	q1_ += qDot2 * (1.0f / frequency_);
	q2_ += qDot3 * (1.0f / frequency_);
	q3_ += qDot4 * (1.0f / frequency_);

	// Normalise quaternion
    recipNorm = 1.0f / sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= recipNorm;
	q1_ *= recipNorm;
	q2_ *= recipNorm;
	q3_ *= recipNorm;
}


}
