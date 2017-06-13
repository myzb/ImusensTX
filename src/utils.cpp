/*
 * utils.cpp
 *
 *  Created on: 13 Jun 2017
 *      Author: matt
 */

#include <i2c_t3.h>

#include "utils.h"
#include "quaternionFilters.h"

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
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
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

void getEulers()
{
    float yaw, pitch, roll;
    float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
    float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)/

    a12 = 2.0f * (*(getQ()+1) * *(getQ()+2) + *(getQ()+0) * *(getQ()+3));

    a22 = *(getQ()+0) * *(getQ()+0) + *(getQ()+1) * *(getQ()+1) - *(getQ()+2) *
          *(getQ()+2) - *(getQ()+3) * *(getQ()+3);

    a31 = 2.0f * (*(getQ()+0) * *(getQ()+1) + *(getQ()+2) * *(getQ()+3));

    a32 = 2.0f * (*(getQ()+1) * *(getQ()+3) - *(getQ()+0) * *(getQ()+2));

    a33 = *(getQ()+0) * *(getQ()+0) - *(getQ()+1) * *(getQ()+1) - *(getQ()+2) *
          *(getQ()+2) + *(getQ()+3) * *(getQ()+3);

    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    yaw   += 2.23f; // Declination at Bandwaldalle, Karlsruhe is 2.23 degrees on 2017-06-06
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / PI;

    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    //lin_ax = ax + a31;
    //lin_ay = ay + a32;
    //lin_az = az - a33;
}

float getTimeDelta(uint32_t *lastUpdate)
{
    uint32_t now;
    float deltat;

    now = micros();
    // Set integration time by time elapsed since last filter update
    deltat = ((now - *lastUpdate) / 1000000.0f);
    *lastUpdate = now;

    //sum += deltat; // sum for averaging filter update rate
    //sumCount++;
    return deltat;
}
