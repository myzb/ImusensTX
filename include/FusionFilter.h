/*
 * FusionFilter.h
 *
 *  Created on: 4 Oct 2017
 *      Author: matt
 */

#ifndef _FUSIONFILTER_H_
#define _FUSIONFILTER_H_

// Complementary Filter Class
class FusionFilter {

private:
    // 3x3 float matrix
    typedef struct mat3 {
        float x[3];
        float y[3];
        float z[3];
    } mat3f_t;

    float _q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

    float VecDot(float *u, float *v);
    float VecNorm(float *v, float *v_out = NULL);
    float QuatNorm(float *q, float *q_out = NULL);
    void VecCross(float *u, float *v, float *cross_out);
    void AxAngle2Quat(float angle, float *axis, float *q_out);
    void QuatMult(float *r, float *q, float *q_out);
    void Quat2Mat(const float *q, mat3f_t *R_out);
    void QuatRot(float *q, float *qv, float *qv_out);
    void VecRot(float *q, float *v, float *v_out);

public:
    float _alpha = 0.5f; // initialised high, allowing fast convergence
    float _beta  = 0.5f; // initialised high, allowing fast convergence

    const float *GetQuat();
    void SetQuat(const float *q_in);
    void Prediction(float *w1_in, float *w2_in, float dt);
    void Correction(float *a1_in, float *m1_in, float *a2_in, float *m2_in, int m1_rdy, int m2_rdy);
};

#endif /* _FUSIONFILTER_H_ */
