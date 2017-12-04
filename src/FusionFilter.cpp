/*
 * FusionFilter.cpp
 *
 *  Created on: 4 Oct 2017
 *      Author: matt
 */

#include <Arduino.h>
#include <arm_math.h>

#include "MetroExt.h"
#include "FusionFilter.h"

// Use Sensor 2 only (disable diferential)
#define SENSOR2_ONLY

// Using ARM optimised trigonometrics
#define ARM_MATH

static float fast_acosf(float x) {
    // Lagrange polynomial of arcos for x=[-1,-0.5,0,0.5,1]
    float angle = (-0.6981f*x*x -0.8727f)*x + 1.5708f;

    if (angle > 1.0f) {
        angle = 1.0f;
    } else if (angle < -1.0f) {
        angle = -1.0f;
    }

    return angle;
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
        // Serial.printf("%s: Division by Zero!\n", __func__); // TODO: do something else?
        return 0.0f;
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

// Transform quaternion to DCM
void FusionFilter::Quat2Mat(const float *q, mat3f_t *R_out)
{
    // Avoid repeated math
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

    // DCM Matrix: xyz = rows, 123 = cols
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

// Rotate vector by rot quaternion
void FusionFilter::VecRot(float *q, float *v, float *v_out)
{
    // Get DCM R(q)
    mat3f_t R;
    Quat2Mat(q, &R);

    // v_out = R(q)*v
    v_out[0] = R.x[0]*v[0] + R.x[1]*v[1] + R.x[2]*v[2];
    v_out[1] = R.y[0]*v[0] + R.y[1]*v[1] + R.y[2]*v[2];
    v_out[2] = R.z[0]*v[0] + R.z[1]*v[1] + R.z[2]*v[2];
}

// Rotate pure quaternion by rot quaternion
void FusionFilter::QuatRot(float *q, float *qv, float *qv_out)
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

    // qv_out = q*qv*q^(-1)
    //qv is a vector quaternion qv = (0, v)

    qv_out[0] = qv[0];
    qv_out[1] = (q0q0 + q1q1 - q2q2 - q3q3)*qv[1] + 2.0f*(q1q2 - q0q3)*qv[2] + 2.0f*(q1q3 + q0q2)*qv[3];
    qv_out[2] = 2.0f*(q1q2 + q0q3)*qv[1] + (q0q0 - q1q1 + q2q2 - q3q3)*qv[2] + 2.0f*(q2q3 - q0q1)*qv[3];
    qv_out[3] = 2.0f*(q1q3 - q0q2)*qv[1] + 2.0f*(q2q3 + q0q1)*qv[2] + (q0q0 - q1q1 - q2q2 + q3q3)*qv[3];

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
        // Serial.printf("%s: Division by Zero!\n", __func__); // TODO: do something else?
        return 0.0f;
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
    // LEGEND:
    // w1: Vehicle sensor angular velocity
    // w2: Head sensor angular velocity
    // dt: time interval

#ifdef SENSOR2_ONLY
    w1_in[0] = w1_in[1] = w1_in[2] = 0.0f;
#endif /* SENSOR2_ONLY */

    // Get last estimated rotation
    float q_in[4] = { _q[0], _q[1], _q[2], _q[3] };

    float w1_norm = VecNorm(w1_in);
    float w2_norm = VecNorm(w2_in);

    // Predict rotation 'q_rot' from 'angle*dt' around 'axis'
    float q1_rot[4], q2_rot[4];
    AxAngle2Quat(-1.0f*w1_norm*dt, w1_in, q1_rot);  // Inverse vehicle to world rotation
    AxAngle2Quat(w2_norm*dt, w2_in, q2_rot);        // Head to world rotation

    // Multiply with previous rotation
    float q_temp[4];
    QuatMult(q_in, q2_rot, q_temp);                  // Head to world in vehicle coords
    QuatMult(q1_rot, q_temp, _q);                    // Head to vehicle in vehicle coords

    // Normalise
    QuatNorm(_q);

    return;
}

void FusionFilter::Correction(float *a1_in, float *m1_in, float *a2_in, float *m2_in,
    uint16_t new_m1_data, uint16_t new_m2_data)
{
    // TODO: Re-Check normalisations
    //       Check NaN's

    // LEGEND:
    // a1, m1: Vehicle sensor data
    // a2, m2: Head sensor data
    // v_ : Vector, q_ : Quaternion
    // V: Vehicle,  H: Head

#ifdef SENSOR2_ONLY
    a1_in[0] = 0.0f; a1_in[1] = 0.0f; a1_in[2] = 1.0f;
    m1_in[0] = 1.0f; m1_in[1] = 0.0f; m1_in[2] = 0.0f;
    new_m1_data = new_m2_data;
#endif /* SENSOR2_ONLY */

    /* Correction step 1: XY-Plane alignment */
    // Get predicted rotation
    float q_in[4] = { _q[0], _q[1], _q[2], _q[3] };

    // Normalise accel data
    VecNorm(a1_in);
    VecNorm(a2_in);

    // Rotate a2_in vector (Head frame) -> v_Va2 (Vehicle frame), v_Va2 is predicted
    float v_Va2[3];
    VecRot(q_in, a2_in, v_Va2);
    VecNorm(v_Va2);

    // Axis orthogonal to vehicle a1_in vector and v_Va2
    float axis[3];
    float dot, angle;

    VecCross(v_Va2, a1_in, axis);
    if (VecNorm(axis) <= 0.0f) goto mag_corr;

    // (Error) Angle between a1_in vector and v_Va2
    dot = VecDot(v_Va2, a1_in);
    angle = fast_acosf(dot);

    // Axis-Angle to innovation quaternion q_a_ino (from acceleration)
    float q_a_ino[4];
    AxAngle2Quat( _alpha*angle, axis, q_a_ino);
    if (QuatNorm(q_a_ino) <= 0.0f) goto mag_corr;

    // Apply 1st correction
    QuatMult(q_a_ino, q_in, _q);
    QuatNorm(_q);

mag_corr:

#if 1
    // Return if no new magnetometer data
    if (!new_m1_data && !new_m2_data) return;
#else
    return;
#endif

    /* Correction step 2: North alignment */
    // Get accel-corrected predicted rotation
    memcpy(q_in, _q, 4*sizeof(float));

    // Normalise mag data
    VecNorm(m1_in);
    VecNorm(m2_in);

    // Rotate m2_in vector (head frame) -> v_Vm2 (vehicle frame)
    float v_Vm2[4];
    VecRot(q_in, m2_in, v_Vm2);
    // Project onto XY Plane
    v_Vm2[2] = 0.0f;
    m1_in[2] = 0.0f;
    VecNorm(v_Vm2);
    VecNorm(m1_in);

    // Axis orthogonal to vehicle m1_in vector and v_Vm2
    VecCross(v_Vm2, m1_in, axis);
    if (VecNorm(axis) <= 0.0f) goto end;

    // (Error) Angle between m1_in vector and v_Vm2
    dot = VecDot(v_Vm2, m1_in);
    angle = fast_acosf(dot);

    // Axis-Angle to innovation quaternion q_m_ino (from magnetic field)
    float q_m_ino[4];
    AxAngle2Quat( _beta*angle, axis, q_m_ino);
    if (QuatNorm(q_m_ino) <= 0.0f) goto end;

    // Apply 2nd correction
    QuatMult(q_m_ino, q_in, _q);
    QuatNorm(_q);

end:
    return;
}

const float *FusionFilter::GetQuat()
{
    return _q;
}

void FusionFilter::SetQuat(const float *q_in)
{
    memcpy(_q, q_in, 4*sizeof(float));
}
