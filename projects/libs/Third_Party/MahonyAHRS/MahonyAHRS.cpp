//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.hpp"
#include <math.h>

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRS::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		updateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = this->q0 * this->q0;
        q0q1 = this->q0 * this->q1;
        q0q2 = this->q0 * this->q2;
        q0q3 = this->q0 * this->q3;
        q1q1 = this->q1 * this->q1;
        q1q2 = this->q1 * this->q2;
        q1q3 = this->q1 * this->q3;
        q2q2 = this->q2 * this->q2;
        q2q3 = this->q2 * this->q3;
        q3q3 = this->q3 * this->q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(this->twoKi > 0.0f) {
			this->integralFBx += this->twoKi * halfex * (1.0f / this->sampleFreq);	// integral error scaled by Ki
			this->integralFBy += this->twoKi * halfey * (1.0f / this->sampleFreq);
			this->integralFBz += this->twoKi * halfez * (1.0f / this->sampleFreq);
			gx += this->integralFBx;	// apply integral feedback
			gy += this->integralFBy;
			gz += this->integralFBz;
		}
		else {
			this->integralFBx = 0.0f;	// prevent integral windup
			this->integralFBy = 0.0f;
			this->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += this->twoKp * halfex;
		gy += this->twoKp * halfey;
		gz += this->twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / this->sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / this->sampleFreq));
	gz *= (0.5f * (1.0f / this->sampleFreq));
	qa = this->q0;
	qb = this->q1;
	qc = this->q2;
	this->q0 += (-qb * gx - qc * gy - this->q3 * gz);
	this->q1 += (qa * gx + qc * gz - this->q3 * gy);
	this->q2 += (qa * gy - qb * gz + this->q3 * gx);
	this->q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(this->q0 * this->q0 + this->q1 * this->q1 + this->q2 * this->q2 + this->q3 * this->q3);
	this->q0 *= recipNorm;
	this->q1 *= recipNorm;
	this->q2 *= recipNorm;
	this->q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRS::updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = this->q1 * this->q3 - this->q0 * this->q2;
		halfvy = this->q0 * this->q1 + this->q2 * this->q3;
		halfvz = this->q0 * this->q0 - 0.5f + this->q3 * this->q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(this->twoKi > 0.0f) {
			this->integralFBx += this->twoKi * halfex * (1.0f / this->sampleFreq);	// integral error scaled by Ki
			this->integralFBy += this->twoKi * halfey * (1.0f / this->sampleFreq);
			this->integralFBz += this->twoKi * halfez * (1.0f / this->sampleFreq);
			gx += this->integralFBx;	// apply integral feedback
			gy += this->integralFBy;
			gz += this->integralFBz;
		}
		else {
			this->integralFBx = 0.0f;	// prevent integral windup
			this->integralFBy = 0.0f;
			this->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += this->twoKp * halfex;
		gy += this->twoKp * halfey;
		gz += this->twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / this->sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / this->sampleFreq));
	gz *= (0.5f * (1.0f / this->sampleFreq));
	qa = this->q0;
	qb = this->q1;
	qc = this->q2;
	this->q0 += (-qb * gx - qc * gy - this->q3 * gz);
	this->q1 += (qa * gx + qc * gz - this->q3 * gy);
	this->q2 += (qa * gy - qb * gz + this->q3 * gx);
	this->q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(this->q0 * this->q0 + this->q1 * this->q1 + this->q2 * this->q2 + this->q3 * this->q3);
	this->q0 *= recipNorm;
	this->q1 *= recipNorm;
	this->q2 *= recipNorm;
	this->q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float MahonyAHRS::invSqrt(float x) {
	// float halfx = 0.5f * x;
	// float y = x;
	// long i = *(long*)&y;
	// i = 0x5f3759df - (i>>1);
	// y = *(float*)&i;
	// y = y * (1.5f - (halfx * y * y));
	// return y;
	float halfx = 0.5f * x;
    union {
        float f;
        long i;
    } conv;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
