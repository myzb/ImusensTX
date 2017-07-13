#ifndef _UTILS_H_
#define _UTILS_H_

typedef union {
  float num_f[16];
  uint32_t num_d[16];
  byte raw[64];
} data_t;

void I2Cscan();
float getTimeDelta(uint32_t *now, uint32_t *lastUpdate);
void quatDiv(float *q, float *r, float *q_out);

#endif /* _UTILS_H_ */
