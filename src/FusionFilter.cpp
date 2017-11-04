/*
 * FusionFilter.cpp
 *
 *  Created on: 4 Nov 2017
 *      Author: matt
 */

/*
 * CompFilter.cpp
 *
 *  Created on: Oct 19, 2017
 *      Author: may
 */

#include <Arduino.h>
#include <arm_math.h>

#include "MetroExt.h"
#include "FusionFilter.h"

// Using ARM optimised trigonometrics
#define ARM_MATH

static float fast_acosf(float x) {
    // Lagrange polynomial of arcos for x=[-1,-0.5,0,0.5,1]
    return (-0.6981f*x*x -0.8727f)*x + 1.5708f;
}

// Vector normalise
float FusionFilter::VecNorm(float *v)
{
#ifdef ARM_MATH
    float norm;
    arm_sqrt_f32(v[0]*v[0] + v[1]*v[1] + v[2]*v[2], &norm);
#else
    float norm = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
#endif

    if (norm == 0.0f) {
//        Serial.printf("%s: Division by Zero!\n", __func__); // TODO: do something
        return -1.0f;
    }
    float factor = 1.0f/norm;

    for (unsigned int i = 0; i < 3; i++)
        v[i] = v[i]*factor;

    return norm;
}

// Vector cross product
void FusionFilter::VecCross(float *u,float *v, float *cross_out)
{
    cross_out[0] = u[1]*v[2] - u[2]*v[1];
    cross_out[1] = u[2]*v[0] - u[0]*v[2];
    cross_out[2] = u[0]*v[1] - u[1]*v[0];
}

// Vector dot product
float FusionFilter::VecDot(float *u, float *v)
{
    // vectors u and v are assumed to be normalised
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

// Transform quaternion to rotation matrix
void FusionFilter::Quat2Mat(const float *q, mat3f_t *R_out)
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

    // The Matrix: xyz = rows, 123 = cols
    R_out->x[0] = q0q0 + q1q1 - q2q2 - q3q3;
    R_out->x[1] = 2.0f*(q1q2 - q0q3);
    R_out->x[2] = 2.0f*(q1q3 + q0q2);

    R_out->y[0] = 2.0f*(q1q2 + q0q3);
    R_out->y[1] = q0q0 - q1q1 + q2q2 - q3q3;
    R_out->y[2] = 2.0f*(q2q3 - q0q1);

    R_out->z[0] = 2.0f*(q1q3 - q0q2);
    R_out->z[1] = 2.0f*(q2q3 + q0q1);
    R_out->z[2] = q0q0 - q1q1 - q2q2 + q3q3;

    return;
}

// Rotate pure quaternion by rot quaternion
void FusionFilter::QuatRot(float *q, float *qv, float *qv_out)
{
#if 0
    // qv_out = R(q)*qv
    mat3f_t R;
    Quat2Mat(q, &R);

    //qv is a vector quaternion qv = (0, vec)
    qv_out[0] = qv[0];
    qv_out[1] = R.x[0]*qv[1] + R.x[1]*qv[2] + R.x[2]*qv[3];
    qv_out[2] = R.y[0]*qv[1] + R.y[1]*qv[2] + R.y[2]*qv[3];
    qv_out[3] = R.z[0]*qv[1] + R.z[1]*qv[2] + R.z[2]*qv[3];
#else
    // qv_out = q*qv*q^(-1)

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

    //qv is a vector quaternion qv = (0, vec)
    qv_out[0] = qv[0];
    qv_out[1] = (q0q0 + q1q1 - q2q2 - q3q3)*qv[1] + 2.0f*(q1q2 - q0q3)*qv[2] + 2.0f*(q1q3 + q0q2)*qv[3];
    qv_out[2] = 2.0f*(q1q2 + q0q3)*qv[1] + (q0q0 - q1q1 + q2q2 - q3q3)*qv[2] + 2.0f*(q2q3 - q0q1)*qv[3];
    qv_out[3] = 2.0f*(q1q3 - q0q2)*qv[1] + 2.0f*(q2q3 + q0q1)*qv[2] + (q0q0 - q1q1 - q2q2 + q3q3)*qv[3];
#endif

    return;
}

// Quaternion product
void FusionFilter::QuatMult(float *r, float *q, float *q_out)
{
    q_out[0] = (r[0]*q[0] - r[1]*q[1] - r[2]*q[2] - r[3]*q[3]);
    q_out[1] = (r[0]*q[1] + r[1]*q[0] + r[2]*q[3] - r[3]*q[2]);
    q_out[2] = (r[0]*q[2] - r[1]*q[3] + r[2]*q[0] + r[3]*q[1]);
    q_out[3] = (r[0]*q[3] + r[1]*q[2] - r[2]*q[1] + r[3]*q[0]);

    return;
}

// Quaternion normalise
float FusionFilter::QuatNorm(float *q)
{
#ifdef ARM_MATH
    float norm;
    arm_sqrt_f32(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] +q[3]*q[3], &norm);
#else
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] +q[3]*q[3]);
#endif

    if (norm == 0.0f) {
//        Serial.printf("%s: Division by Zero!\n", __func__); // TODO: do something
        return -1.0f;
    }
    float factor = 1.0f/norm;

    for (unsigned int i = 0; i < 4; i++)
        q[i] = q[i]*factor;

    return norm;
}

// Transform axis-angle to quaternion representation
void FusionFilter::AxAngle2Quat(float angle, float *axis, float *q_out)
{
#ifdef ARM_MATH
    float sin_ = arm_sin_f32(0.5f*angle);
    q_out[0] = arm_cos_f32(0.5f*angle);
#else
    float sin_ = sinf(0.5f*angle);
    q_out[0] = cosf(0.5f*angle);
#endif
    q_out[1] = axis[0]*sin_;
    q_out[2] = axis[1]*sin_;
    q_out[3] = axis[2]*sin_;

    return;
}

void FusionFilter::Prediction(float *w1_in, float *w2_in, float dt)
{
    float q_in[4] = { _q[0], _q[1], _q[2], _q[3] };
    float q_out[4];

    float w1_norm = VecNorm(w1_in);
    float w2_norm = VecNorm(w2_in);

    // Predict rotation 'q_rot' from 'angle*dt' around 'axis'
    float q1_rot[4], q2_rot[4];
    AxAngle2Quat(w1_norm*dt, w1_in, q1_rot);        // head to world rotation
    AxAngle2Quat(-1.0f*w2_norm*dt, w2_in, q2_rot);  // Inverse vehicle to world rotation

    // Multiply with previous rotation
    QuatMult(q_in, q2_rot, q_out);                  // Head to world in vehicle coords
    QuatMult(q1_rot, q_out, _q);                    // Head to vehicle in vehicle coords

    // Normalise
    QuatNorm(_q);

    return;
}

void FusionFilter::Correction(float *a1_in, float *m1_in, float *a2_in, float *m2_in,
    uint16_t new_m1, uint16_t new_m2)
{
    // TODO: Check where normalisation is not needed
    //       Check NaN's

    // LEGEND:
    // a1, m1: Vehicle sensor data
    // a2, m2: Head sensor data

    float *q_in = _q;
    float q_out[4] = { _q[0], _q[1], _q[2], _q[3] };

    /* Correction step 1: XY-Plane alignment */

    // Normalise accel data
    VecNorm(a1_in);
    VecNorm(a2_in);

    float q_Sa2[4] = { 0.0f, a2_in[0], a2_in[1], a2_in[2] };

    // Rotate q_Sa2 (Head frame) -> q_Va (Vehicle frame)
    float q_Va2[4];
    QuatRot(q_in, q_Sa2, q_Va2);
    QuatNorm(q_Va2);

    // Axis orthogonal to vehicle accel_vec and q_Va2 = [0, vec3_Va2]
    float axis[3];
    float dot, angle;

    VecCross(q_Va2+1, a1_in, axis);
    if (VecNorm(axis) < 0.0f) goto mag_corr;

    // (Error) Angle between gravity vector and q_Ea
    dot = VecDot(q_Va2+1, a1_in);
    angle = fast_acosf(dot);

    // Axis-Angle to correction quaternion q_c1
    float q_c1[4];
    AxAngle2Quat( _alpha*angle, axis, q_c1);
    if (QuatNorm(q_c1) < 0.0f) goto mag_corr;

    // Apply 1st correction
    QuatMult(q_c1, q_in, q_out);
    QuatNorm(q_out);

    // Return if no new magnetometer data
    mag_corr:
#if 1
    if (!new_m1 && !new_m2) {
        memcpy(_q, q_out, 4*sizeof(float));
        return;
    }
#else
    memcpy(_q, q_out, 4*sizeof(float));
    return;
#endif

    /* Correction step 2: North alignment */
    q_in = q_out;

    // Normalise mag data
    VecNorm(m1_in);
    VecNorm(m2_in);
    float q_Sm2[4] = { 0.0f, m2_in[0], m2_in[1], m2_in[2] };

    // Rotate q_Sm2 (head frame) -> q_Vm2 (vehicle frame)
    float q_Vm2[4];
    QuatRot(q_in, q_Sm2, q_Vm2);
    // Project onto XY Plane
    q_Vm2[3] = 0.0f;
    m1_in[2] = 0.0f;
    QuatNorm(q_Vm2);
    VecNorm(m1_in);

    // Axis orthogonal to north vector and q_Em = [0, vec3_Em]
    VecCross(q_Vm2+1, m1_in, axis);
    if (VecNorm(axis) < 0.0f) goto end;

    // (Error) Angle between north vector and q_Em
    dot = VecDot(q_Vm2+1, m1_in);
    angle = fast_acosf(dot);

    // Axis-Angle to correction quaternion q_c1
    float q_c2[4];
    AxAngle2Quat( (_beta)*angle, axis, q_c2);
    if (QuatNorm(q_c2) < 0.0f) goto end;

    // Apply 2nd correction
    QuatMult(q_c2, q_in, _q);
    QuatNorm(_q);

    return;

end:
    memcpy(_q, q_out, 4*sizeof(float));
    return;
}

const float *FusionFilter::GetQuat()
{
    return _q;
}
