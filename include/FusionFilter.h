#ifndef _FUSIONFILTER_H_
#define _FUSIONFILTER_H_

#include "mpu9250.h"

class FusionFilter {

public:

    // These are the free parameters in the Mahony filter and fusion scheme, Kp
    // for proportional feedback, Ki for integral
    const float Kp = 2.0f * 2.0f;
    const float Ki = 0.001f;

    // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    const float GyroMeasError = PI * (20.0f / 180.0f);

    const float GyroMeasDrift = PI * (0.0f / 180.0f);
    // There is a tradeoff in the beta parameter between accuracy and response
    // speed. In the original Madgwick study, beta of 0.041 (corresponding to
    // GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
    // However, with this value, the LSM9SD0 response time is about 10 seconds
    // to a stable initial quaternion. Subsequent changes also require a
    // longish lag time to a stable output, not fast enough for a quadcopter or
    // robot car! By increasing beta (GyroMeasError) by about a factor of
    // fifteen, the response time constant is reduced to ~2 sec. I haven't
    // noticed any reduction in solution accuracy. This is essentially the I
    // coefficient in a PID control sense; the bigger the feedback coefficient,
    // the faster the solution converges, usually at the expense of accuracy.
    // In any case, this is the free parameter in the Madgwick filtering and
    // fusion scheme.
    const float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
    // Compute zeta, the other free parameter in the Madgwick scheme usually
    // set to a small or zero value
    const float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

    // Vector to hold integral error for Mahony method
    float eInt[3] = { 0.0f, 0.0f, 0.0f };
    // Vector to hold quaternion
    float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

    void MadgwickUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx,
        float my, float mz, float dt);
    void MahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx,
        float my, float mz, float dt);
    const float *GetQuat();
};

#endif /* _FUSIONFILTER_H_ */
