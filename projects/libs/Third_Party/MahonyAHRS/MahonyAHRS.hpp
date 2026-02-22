//=====================================================================================================
// MahonyAHRS.h
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
#pragma once

//----------------------------------------------------------------------------------------------------
// MahonyAHRS class declaration

class MahonyAHRS {
public:
    /**
     * @brief Construct a new Mahony AHRS object
     * 
     * @param sampleFreq sample frequency in Hz
     * @param twoKp 2 * proportional gain (Kp)
     * @param twoKi 2 * integral gain (Ki) (default is 0, no integral feedback and avoids integral windup)
     */
    MahonyAHRS(float sampleFreq = 512.0f, float twoKp = (2.0f * 0.5f), float twoKi = (2.0f * 0.0f)) :
    sampleFreq(sampleFreq), twoKp(twoKp), twoKi(twoKi), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
    integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f)
    {}

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    private:
        float sampleFreq;
        float twoKp;    // 2 * proportional gain (Kp)
        float twoKi;    // 2 * integral gain (Ki)
        float q0, q1, q2, q3;                       // quaternion of sensor frame relative to auxiliary frame
        float integralFBx, integralFBy, integralFBz; // integral error terms scaled by Ki

        float invSqrt(float x);
};

//=====================================================================================================
// End of file
//=====================================================================================================
