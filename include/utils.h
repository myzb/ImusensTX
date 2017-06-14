#ifndef _UTILS_H_
#define _UTILS_H_

typedef union {
  float num[10];
  byte raw[40];
} data_t;


void I2Cscan();
void getEulers(float *eulers, float *a_gravity);
void dumpData(float ax, float ay, float az, float gx, float gy, float gz,
              float mx, float my, float mz, uint16_t temp);
float getTimeDelta(uint32_t *now, uint32_t *lastUpdate);

#endif /* _UTILS_H_ */
