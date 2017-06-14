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

void getEulers(float *eulers, float *a_gravity)
{
    float a12, a22, a31, a32, a33;// rotation matrix coeffs for Euler angles and gravity components

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    //Software AHRS:

    a12 = 2.0f * (*(getQ()+1) * *(getQ()+2) + *(getQ()+0) * *(getQ()+3));

    a22 = *(getQ()+0) * *(getQ()+0) + *(getQ()+1) * *(getQ()+1) - *(getQ()+2) *
          *(getQ()+2) - *(getQ()+3) * *(getQ()+3);

    a31 = 2.0f * (*(getQ()+0) * *(getQ()+1) + *(getQ()+2) * *(getQ()+3));

    a32 = 2.0f * (*(getQ()+1) * *(getQ()+3) - *(getQ()+0) * *(getQ()+2));

    a33 = *(getQ()+0) * *(getQ()+0) - *(getQ()+1) * *(getQ()+1) - *(getQ()+2) *
          *(getQ()+2) + *(getQ()+3) * *(getQ()+3);

    eulers[0] = -asinf(a32);
    eulers[1] = atan2f(a31, a33);
    eulers[2] = atan2f(a12, a22);

    // Convert do deg
    eulers[0] *= 180.0f / PI;
    eulers[0] += 2.23f;           // Declination at Work, Karlsruhe: 2.23
    if(eulers[0] < 0)
        eulers[0] += 360.0f;        // Ensure yaw stays between 0 and 360 deg

    eulers[1] *= 180.0f / PI;
    eulers[2] *= 180.0f / PI;

    // return a_gravity coeffs
    a_gravity[0] = a31;
    a_gravity[2] = a32;
    a_gravity[1] = a33;
}

void dumpData(float ax, float ay, float az, float gx, float gy, float gz,
              float mx, float my, float mz, uint16_t temp)
{
    static float eulers[3], a_gravity[3], tempDegC;

    // Print basic data
    Serial.print("ax = "); Serial.print((int)1000*ax);
    Serial.print(" ay = "); Serial.print((int)1000*ay);
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2);
    Serial.print(" gy = "); Serial.print( gy, 2);
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mx );
    Serial.print(" my = "); Serial.print( (int)my );
    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");

    // Print gyro temperature in degrees centigrade
    tempDegC = ((float) temp) / 333.87 + 21.0;
    Serial.print("Gyro temperature is ");  Serial.print(tempDegC, 1);  Serial.println(" degrees C");

#ifdef AHRS
    Serial.print("q0 = ");  Serial.print(*(getQ()+0));
    Serial.print(" qx = "); Serial.print(*(getQ()+1));
    Serial.print(" qy = "); Serial.print(*(getQ()+2));
    Serial.print(" qz = "); Serial.println(*(getQ()+3));

    // Quaternion to euler angles conversion
    getEulers(eulers, a_gravity);
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(eulers[0], 2);
    Serial.print(", ");
    Serial.print(eulers[1], 2);
    Serial.print(", ");
    Serial.println(eulers[2], 2);

    Serial.print("Grav_x, Grav_y, Grav_z: ");
    Serial.print(-a_gravity[0]*1000, 2);
    Serial.print(", ");
    Serial.print(-a_gravity[1]*1000, 2);
    Serial.print(", ");
    Serial.print(a_gravity[2]*1000, 2);  Serial.println(" mg");

    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
    Serial.print(ax*a_gravity[0]*1000, 2);
    Serial.print(", ");
    Serial.print(ay*a_gravity[1]*1000, 2);
    Serial.print(", ");
    Serial.print(az*a_gravity[2]*1000, 2);  Serial.println(" mg");
#endif
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
