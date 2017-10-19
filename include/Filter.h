/*
 * Filter.h
 *
 *  Created on: Oct 19, 2017
 *      Author: may
 */

#ifndef _FILTER_H_
#define _FILTER_H_

// Complementary Filter Class
class CompFilter {

private:
    // 3x3 float matrix
    typedef struct {
        float x[3];
        float y[3];
        float z[3];
    } mat3f_t;

    float _q[4]         = { 1.0f, 0.0f, 0.0f, 0.0f };
    float _v_gravity[3] = { 0.0f, 0.0f, 1.0f };
    float _v_north[3]   = { 1.0f, 0.0f, 0.0f };

    const float _alpha = 0.001f;
    const float _beta  = 0.005f;

    // Adaptive gain thersholds
    const float _gain_max = 0.01f;
    const float _th1 = 0.001f;
    const float _th2 = 0.005f;
    const float _G = 9.99f;    // Force of gravity

    float Gain(float *a_in);

    float VecDot(float *u, float *v);
    float VecNorm(float *v);
    float QuatNorm(float *q);
    void VecCross(float *u, float *v, float *cross_out);
    void AxAngle2Quat(float angle, float *axis, float *q_out);
    void QuatMult(float *r, float *q, float *q_out);
    void Quat2Mat(const float *q, mat3f_t *R_out);
    void QuatRot(float *q, float *qv, float *qv_out);

public:
    const float *GetQuat();
    void Prediction(float *w_in, float dt);
    void Correction(float *a_in, float *m_in, uint16_t new_mag);
};

#endif /* _FILTER_H_ */
