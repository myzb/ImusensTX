/*
 * ComplementaryFilter.h
 *
 *  Created on: Sep 20, 2017
 *      Author: may
 */

#ifndef _COMPLEMENTARYFILTER_H_
#define _COMPLEMENTARYFILTER_H_

// 3x3 float Matrix
typedef struct {
    float x[3];
    float y[3];
    float z[3];
} mat3f_t;

// Complementary Filter Class
class Filter {
public:
    float _q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float _alpha = 0.05f;
    const float _beta = 0.05f;

    int VecNorm(float *v, float *v_out);

    void QuatMult(float *r, float *q, float *q_out);
    int QuatNorm(float *q, float *q_out);
    void Quat2Mat(const float *q, mat3f_t *R_out);

    const float *GetQuat();
    void Prediction(float dt, float *w_in);
    void Correction(float *a_in, float *m_in);
};
#endif /* _COMPLEMENTARYFILTER_H_ */
