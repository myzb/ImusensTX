/*
 * ComplementaryFilter.cpp
 *
 *  Created on: Sep 20, 2017
 *      Author: may
 */

#include <Arduino.h>

#include "ComplementaryFilter.h"

int Filter::VecNorm(float *v, float *v_out)
{
    float sqrt_vv = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    if (sqrt_vv == 0.0f) Serial.printf("Division by Zero!\n");

    for (unsigned int i = 0; i < 3; i++)
        v_out[i] = v[i]/sqrt_vv;

    return 1;
}

// Quaternion product
void Filter::QuatMult(float *r, float *q, float *q_out)
{
#if 1
    /* Sabatini */
    q_out[0] = (r[0]*q[0] - r[1]*q[1] - r[2]*q[2] - r[3]*q[3]);
    q_out[1] = (r[0]*q[1] + r[1]*q[0] + r[2]*q[3] - r[3]*q[2]);
    q_out[2] = (r[0]*q[2] - r[1]*q[3] + r[2]*q[0] + r[3]*q[1]);
    q_out[3] = (r[0]*q[3] + r[1]*q[2] - r[2]*q[1] + r[3]*q[0]);
#else
    /* Mathworks */
    q_out[0] = (r[0]*q[0] - r[1]*q[1] - r[2]*q[2] - r[3]*q[3]);
    q_out[1] = (r[0]*q[1] + r[1]*q[0] - r[2]*q[3] + r[3]*q[2]);
    q_out[2] = (r[0]*q[2] + r[1]*q[3] + r[2]*q[0] - r[3]*q[1]);
    q_out[3] = (r[0]*q[3] - r[1]*q[2] + r[2]*q[1] + r[3]*q[0]);
#endif
}

// Quaternion normalize
int Filter::QuatNorm(float *q, float *q_out)
{
    float sqrt_qq = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] +q[3]*q[3]);

    if (sqrt_qq == 0.0f) Serial.printf("Division by Zero!\n");

    for (unsigned int i = 0; i < 4; i++)
        q_out[i] = q[i]/sqrt_qq;

    return 1;
}

//static mat3f_t R;

void Filter::Quat2Mat(const float *q, mat3f_t *R_out)
{
    // q0 * qx
    float q0q0 = q[0]*q[0];
    float q0q1 = q[0]*q[1];
    float q0q2 = q[0]*q[2];
    float q0q3 = q[0]*q[3];

    // q1 * qx
    float q1q1 = q[1]*q[1];
    float q1q2 = q[1]*q[2];
    float q1q3 = q[1]*q[3];

    // q2 * qx
    float q2q2 = q[2]*q[2];
    float q2q3 = q[2]*q[3];

    // q3 * qx
    float q3q3 = q[3]*q[3];

    // The Rotation matrix: xyz = rows, 123 = cols
#if 1
    /* Sabatini */
    R_out->x[0] = q0q0+q1q1-q2q2-q3q3;
    R_out->x[1] = 2.0f*(q1q2-q0q3);
    R_out->x[2] = 2.0f*(q1q3+q0q2);

    R_out->y[0] = 2.0f*(q1q2+q0q3);
    R_out->y[1] = q0q0-q1q1+q2q2-q3q3;
    R_out->y[2] = 2.0f*(q2q3-q0q1);

    R_out->z[0] = 2.0f*(q1q3-q0q2);
    R_out->z[1] = 2.0f*(q2q3+q0q1);
    R_out->z[2] = q0q0-q1q1-q2q2+q3q3;
#else
    /* Mathworks */
    R_out->x[0] = 1.0f - 2.0f*q2q2 - 2.0f*q3q3;
    R_out->x[1] = 2.0f*(q1q2 + q0q3);
    R_out->x[2] = 2.0f*(q1q3 - q0q2);

    R_out->y[0] = 2.0f*(q1q2 - q0q3);
    R_out->y[1] = 1.0f - 2.0f*q1q1 - 2.0f*q3q3;
    R_out->y[2] = 2.0f*(q2q3 + q0q1);

    R_out->z[0] = 2.0f*(q1q3 + q0q2);
    R_out->z[1] = 2.0f*(q2q3 - q0q1);
    R_out->z[2] = 1.0f - 2.0f*q1q1 - 2.0f*q2q2;
#endif
}

void Filter::Prediction(float dt, float *w_in)
{
    float q_in[4] = { _q[0], _q[1], _q[2], _q[3] };

    // Arange angular velocity as pure quaternion
    float wquat[4] = { 0.0f, w_in[0], w_in[1], w_in[2] };

    //if (!quatNorm(wquat, wquat)) return;
    //QuatNorm(wquat, wquat);

/*
    Serial.printf("%s: qq_1 = \t%.2f\t%.2f\t%.2f\t%.2f\n", __func__,  wquat[0], wquat[1], wquat[2], wquat[3]);
    Serial.printf("%s: qq_2 = \t%.2f\t%.2f\t%.2f\t%.2f\n", __func__,  q_in[0], q_in[1], q_in[2], q_in[3]);
    Serial.printf("%s: qq_3 = \t%.2f\t%.2f\t%.2f\t%.2f\n", __func__,  q_out[0], q_out[1], q_out[2], q_out[3]);
*/

    // Compute derivate of rotation quaternion and integrate over dt
    //      dquat = -0.5 wquat * q_in
    //      q_out = q_in + dquat * dt
    float q_res[4];
    QuatMult(wquat, q_in, q_res);

    for (unsigned int i = 0; i < 4; i++)
        _q[i] =  q_in[i] - 0.5f * q_res[i] * dt;

    //Serial.printf("%s: _q = \t%.2f\t%.2f\t%.2f\t%.2f\n", __func__,  _q[0], _q[1], _q[2], _q[3]);
}

void Filter::Correction(float *a_in, float *m_in)
{
    float q_in[4] = { _q[0], _q[1], _q[2], _q[3] };

    // (1) Acceleromenter
    // Calculate the rotation matrix
    mat3f_t R;
    Quat2Mat(q_in, &R);

    float a[3];
    VecNorm(a_in, a);   // Normalise accel data

    // Predicted gravity 'g' = R_transposed * a
    float g[3] = {
                  R.x[0] * a[0] + R.y[0] * a[1] + R.z[0] * a[2],
                  R.x[1] * a[0] + R.y[1] * a[1] + R.z[1] * a[2],
                  R.x[2] * a[0] + R.y[2] * a[1] + R.z[2] * a[2]
    };

    //Serial.printf("%s: g[] = \t%.2f\t%.2f\t%.2f\n", __func__, g[0], g[1], g[2]);

    // Delta quaternion q_acc
    float dq_acc[4] = {
                       sqrtf( (g[2] + 1.0f)/2.0f ),
                      -g[1] / sqrtf( 2.0f*(g[2]+1.0f) ),
                       g[0] / sqrtf( 2.0f*(g[2]+1.0f) ),
                       0.0f
    };
/*
    Serial.printf("%s: dq_acc[] = \t%.2f\t%.2f\t%.2f\t%.2f\n", __func__, dq_acc[0], dq_acc[1], dq_acc[2], dq_acc[3]);
    static int counter = 0;
    counter++;
    if (counter == 20) exit(1);
*/
    // Acc LERP
    float dq_acc_[4] = {
        (1.0f - _alpha) + _alpha*dq_acc[0],
        _alpha*dq_acc[1],
        _alpha*dq_acc[2],
        _alpha*dq_acc[3]
    };

    // Normalize the quaternion
    float nq_acc[4];
    QuatNorm(dq_acc_, nq_acc);

    // Applying 1st correction
    float q_new[4];
    QuatMult(q_in, nq_acc, q_new);

#if 1
    // (2) Magnetometer
    // Calculate the new rotation matrix
    Quat2Mat(q_new, &R);

    float m[3];
    VecNorm(m_in, m);   // Normalise mag data

    // magnetic vector into world frame: R_transposed * m
    float l[3] = {
                  R.x[0] * m[0] + R.y[0] * m[1] + R.z[0] * m[2],
                  R.x[1] * m[0] + R.y[1] * m[1] + R.z[1] * m[2],
                  R.x[2] * m[0] + R.y[2] * m[1] + R.z[2] * m[2]
    };
    float Gma = l[0]*l[0] + l[1]*l[1];

    // Delta quaternion q_mag
    float dq_mag[4] = {
       sqrtf( (Gma + l[0]*sqrtf(Gma)) / (2.0f*Gma) ),
       0.0f,
       0.0f,
       l[1] / sqrtf( 2.0f*(Gma + l[0]*sqrtf(Gma)) )
    };

    // Mag LERP
    float dq_mag_[4] = {
        (1.0f - _beta) + _beta*dq_mag[0],
        _beta*dq_mag[1],
        _beta*dq_mag[2],
        _beta*dq_mag[3]
    };

    // Normalize the quaternion
    float nq_mag[4];
    QuatNorm(dq_mag_, nq_mag);

    // Applying 2st correction
    QuatMult(q_new, nq_mag, _q);
#else
    memcpy(_q, q_new, 4*sizeof(float));
#endif
}

const float *Filter::GetQuat()
{
    return _q;
}
