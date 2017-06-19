#ifndef _UTILS_H_
#define _UTILS_H_

typedef union {
  float num_f[16];
  uint32_t num_d[16];
  byte raw[64];
} data_t;


void I2Cscan();
void getEulers(float *eulers, float *a_gravity);
void dumpData(float ax, float ay, float az, float gx, float gy, float gz,
              float mx, float my, float mz, uint16_t temp);
float getTimeDelta(uint32_t *now, uint32_t *lastUpdate);

#endif /* _UTILS_H_ */
