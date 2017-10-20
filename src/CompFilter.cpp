/*
 * CompFilter.cpp
 *
 *  Created on: Oct 19, 2017
 *      Author: may
 */

#include <Arduino.h>
#include <arm_math.h>

#include "MetroExt.h"
#include "CompFilter.h"

#define ARM_MATH

// Vector normalise
float CompFilter::VecNorm(float *v)
{
#ifdef ARM_MATH
    float norm;
    arm_sqrt_f32(v[0]*v[0] + v[1]*v[1] + v[2]*v[2], &norm);
#else
    float norm = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
#endif

    if (norm == 0.0f) Serial.printf("Division by Zero!\n"); // TODO: do something
    float factor = 1.0f/norm;

    for (unsigned int i = 0; i < 3; i++)
        v[i] = v[i]*factor;

    return norm;
}

// Vector cross product
void CompFilter::VecCross(float *u,float *v, float *cross_out)
{
    cross_out[0] = u[1]*v[2] - u[2]*v[1];
    cross_out[1] = u[2]*v[0] - u[0]*v[2];
    cross_out[2] = u[0]*v[1] - u[1]*v[0];
}

// Vector dot product
float CompFilter::VecDot(float *u, float *v)
{
    // vectors u and v are assumed to be normalised
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

// Transform quaternion to rotation matrix
void CompFilter::Quat2Mat(const float *q, mat3f_t *R_out)
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
void CompFilter::QuatRot(float *q, float *qv, float *qv_out)
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
void CompFilter::QuatMult(float *r, float *q, float *q_out)
{
    q_out[0] = (r[0]*q[0] - r[1]*q[1] - r[2]*q[2] - r[3]*q[3]);
    q_out[1] = (r[0]*q[1] + r[1]*q[0] + r[2]*q[3] - r[3]*q[2]);
    q_out[2] = (r[0]*q[2] - r[1]*q[3] + r[2]*q[0] + r[3]*q[1]);
    q_out[3] = (r[0]*q[3] + r[1]*q[2] - r[2]*q[1] + r[3]*q[0]);

    return;
}

// Quaternion normalise
float CompFilter::QuatNorm(float *q)
{
#ifdef ARM_MATH
    float norm;
    arm_sqrt_f32(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] +q[3]*q[3], &norm);
#else
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] +q[3]*q[3]);
#endif

    if (norm == 0.0f) Serial.printf("Division by Zero!\n"); // TODO: do something
    float factor = 1.0f/norm;

    for (unsigned int i = 0; i < 4; i++)
        q[i] = q[i]*factor;

    return norm;
}

// Transform axis-angle to quaternion representation
void CompFilter::AxAngle2Quat(float angle, float *axis, float *q_out)
{
#ifdef ARM_MATH
    // Using ARM optimised trigonometrics
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

void CompFilter::Prediction(float *w_in, float dt)
{
    float q_in[4] = { _q[0], _q[1], _q[2], _q[3] };

    float w_norm = VecNorm(w_in);

    // Predict rotation 'q_rot' from 'angle*dt' around 'axis'
    float q_rot[4];
    AxAngle2Quat(w_norm*dt, w_in, q_rot);

    // Multiply with previous rotation
    QuatMult(q_in, q_rot, _q);

    // Normalise
    QuatNorm(_q);

    return;
}

void CompFilter::Correction(float *a_in, float *m_in, uint16_t new_mag)
{
    // TODO: Check where normalisation is not needed

    float *q_in = _q;
    float q_out[4];

    /* Correction step 1: XY-Plane alignment */

    // Normalise accel data
    VecNorm(a_in);
    float q_Sa[4] = { 0.0f, a_in[0], a_in[1], a_in[2] };

    // Rotate q_Sa (accelSensor frame) -> q_Ea (accelEarth frame)
    float q_Ea[4];
    QuatRot(q_in, q_Sa, q_Ea);
    QuatNorm(q_Ea);

    // Axis orthogonal to gravity vector and q_Ea = [0, vec3_Ea]
    float axis[3];
    VecCross(q_Ea+1, _v_gravity, axis);
    VecNorm(axis);

    // (Error) Angle between gravity vector and q_Ea
    float dot = q_Ea[3];
    float angle = acosf(dot);

    // Axis-Angle to correction quaternion q_c1
    float q_c1[4];
    AxAngle2Quat( _alpha*angle, axis, q_c1);
//    QuatNorm(q_c1);

    // Apply 1st correction
    QuatMult(q_c1, q_in, q_out);
    QuatNorm(q_out);

    // Check NaN
    if (q_out[0] != q_out[0]) {
        Serial.printf("q_Sa = %6.3f\%6.3f\t%6.3f\t%6.3f\n", q_Sa[0], q_Sa[1], q_Sa[2], q_Sa[3]);
        Serial.printf("q_Ea = %6.3f\%6.3f\t%6.3f\t%6.3f\n", q_Ea[0], q_Ea[1], q_Ea[2], q_Ea[3]);
        Serial.printf("q_c1 = %6.3f\%6.3f\t%6.3f\t%6.3f\n", q_c1[0], q_c1[1], q_c1[2], q_c1[3]);
        Serial.printf("angle = %6.3f, axis = %6.3f\%6.3f\t%6.3f\n", angle, axis[0], axis[1], axis[2]);
    }

    // Return if no new magnetometer data
    if (!new_mag) {
        memcpy(_q, q_out, 4*sizeof(float));
        return;
    }

    /* Correction step 2: North alignment */
    q_in = q_out;

    // Normalise mag data
    VecNorm(m_in);
    float q_Sm[4] = { 0.0f, m_in[0], m_in[1], m_in[2] };

    // Rotate q_Sm (magSensor frame) -> q_Em (magEarth frame)
    float q_Em[4];
    QuatRot(q_in, q_Sm, q_Em);
    // Project onto XY Plane
    q_Em[3] = 0.0f;
    QuatNorm(q_Em);

    // Axis orthogonal to north vector and q_Em = [0, vec3_Em]
    VecCross(q_Em+1, _v_north, axis);
    VecNorm(axis);

    // (Error) Angle between north vector and q_Em
    dot = q_Em[1];
    angle = acosf(dot);

    // Axis-Angle to correction quaternion q_c1
    float q_c2[4];
    AxAngle2Quat( (_beta)*angle, axis, q_c2);
//    QuatNorm(q_c2);

    // Apply 2nd correction
    QuatMult(q_c2, q_in, _q);
    QuatNorm(_q);

    // Check NaN
    if (_q[0] != _q[0]) {
        Serial.printf("q_Sm = %6.3f\%6.3f\t%6.3f\t%6.3f\n", q_Sm[0], q_Sm[1], q_Sm[2], q_Sm[3]);
        Serial.printf("q_Em = %6.3f\%6.3f\t%6.3f\t%6.3f\n", q_Em[0], q_Em[1], q_Em[2], q_Em[3]);
        Serial.printf("q_c2 = %6.3f\%6.3f\t%6.3f\t%6.3f\n", q_c2[0], q_c2[1], q_c2[2], q_c2[3]);
        Serial.printf("angle = %6.3f, axis = %6.3f\%6.3f\t%6.3f\n", angle, axis[0], axis[1], axis[2]);
    }

    return;
}

const float *CompFilter::GetQuat()
{
    return _q;
}
