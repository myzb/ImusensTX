/*
 * utils.cpp
 *
 *  Created on: 13 Jun 2017
 *      Author: may
 */

#include <ExtFilter.h>
#include <i2c_t3.h>

#include "utils.h"

// I2C scan function
void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of the Write.endTransmisstion to
    // see if a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

}

float getTimeDelta(uint32_t *lastUpdate)
{
    uint32_t now;
    float deltat;

    now = micros();
    // Set integration time by time elapsed since last filter update
    deltat = ((now - *lastUpdate) / 1000000.0f);
    *lastUpdate = now;

    return deltat;
}

// Quaternion division
void quatDiv(const float *q, const float *r, float *q_out)
{
    // q_out = result = q/r
    float q_rr = r[0]*r[0] + r[1]*r[1] + r[2]*r[2] +r[3]*r[3];

    q_out[0] = (r[0]*q[0] + r[1]*q[1] + r[2]*q[2] + r[3]*q[3])/q_rr;
    q_out[1] = (r[0]*q[1] - r[1]*q[0] - r[2]*q[3] + r[3]*q[2])/q_rr;
    q_out[2] = (r[0]*q[2] + r[1]*q[3] - r[2]*q[0] - r[3]*q[1])/q_rr;
    q_out[3] = (r[0]*q[3] - r[1]*q[2] + r[2]*q[1] - r[3]*q[0])/q_rr;
}
