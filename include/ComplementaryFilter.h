/*
 * ComplementaryFilter.h
 *
 *  Created on: Sep 20, 2017
 *      Author: may
 */

#ifndef _COMPLEMENTARYFILTER_H_
#define _COMPLEMENTARYFILTER_H_

// Complementary Filter Class
class Filter {

private:
    // 3x3 float matrix
    typedef struct {
        float x[3];
        float y[3];
        float z[3];
    } mat3f_t;

    float _q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    float _q_id[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // Identity quaternion
    const float _alpha = 0.001f;
    const float _beta = 0.001f;

    void Lerp(float *q, float *r, float factor, float *q_out);
    void Slerp(float *q, float *r, float factor, float cosRads, float *q_out);

    int VecNorm(float *v, float *v_out);
    void QuatMult(float *r, float *q, float *q_out);
    int QuatNorm(float *q, float *q_out);
    void Quat2Mat(const float *q, mat3f_t *R_out);

public:
    const float *GetQuat();
    void Prediction(float *w_in, float dt);
    void Correction(float *a_in, float *m_in);
};
#endif /* _COMPLEMENTARYFILTER_H_ */
