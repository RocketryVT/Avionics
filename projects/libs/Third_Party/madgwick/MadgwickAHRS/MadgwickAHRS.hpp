//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#pragma once

//----------------------------------------------------------------------------------------------------
// MadgwickAHRS class declaration

class MadgwickAHRS {
public:
    /**
     * @brief Construct a new Madgwick AHRS object
     * 
     * @param sampleFreq sample frequency in Hz
     * @param beta 2 * proportional gain (Kp)
     */
	MadgwickAHRS(float sampleFreq = 512.0f, float beta = 0.1f) : 
	sampleFreq(sampleFreq), beta(beta), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) 
	{}

	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

	// Getters for quaternion
    // quaternion of sensor frame relative to auxiliary frame
	float getQ0() const { return q0; }
	float getQ1() const { return q1; }
	float getQ2() const { return q2; }
	float getQ3() const { return q3; }

	// Setters for beta and sampleFreq
	void setBeta(float b) { beta = b; }
	void setSampleFreq(float sf) { sampleFreq = sf; }

private:
	float beta;
	float sampleFreq;
	float q0, q1, q2, q3;

	float invSqrt(float x);
};

//=====================================================================================================
// End of file
//=====================================================================================================
