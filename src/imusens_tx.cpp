/* MPU9250_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is intended specifically for the MPU9250+MS5637 Add-on shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the MPU9250+MS5637 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
//#include "Wire.h"   
#include <i2c_t3.h>
#include <SPI.h>

#include "utils.h"
#include "quaternionFilters.h"
#include "mpu9250.h"

static const bool SerialDebug = true;  // set to true to get Serial output for debugging

// Pin definitions
int intPin = 33;
volatile bool newData = false;
int myLed = 13;

mpu9250 myImu;

void setup()
{
    float gyroBias[3]; //, accelBias; // FIXME: Move mpu9250::_accelBias here
    float selfTest[6];
    float magCalFactory[3];

//  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);
  Serial.begin(115200);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  I2Cscan();// look for I2C devices on the bus
    
  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MPU9250 9-axis motion sensor...");
  byte c = myImu.ReadByte(myImu.MPU9250_ADDRESS, myImu.WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU9250 is online...");
    
    myImu.SelfTest(selfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(selfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(selfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(selfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(selfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(selfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(selfTest[5],1); Serial.println("% of factory value");
    delay(1000);
    
   // set sensor resolutions, only need to do this once
   myImu.SetAres(mpu9250::AFS_2G);
   myImu.SetGres(mpu9250::GFS_250DPS);
   myImu.SetMres(mpu9250::MFS_16BITS);
   myImu.SetMmode(mpu9250::MRATE_100HZ);
    
   Serial.println(" Calibrate gyro and accel");
   myImu.AcelGyroCal(gyroBias, myImu._accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
   Serial.println("accel biases (mg)"); Serial.println(1000.*myImu._accelBias[0]); Serial.println(1000.*myImu._accelBias[1]); Serial.println(1000.*myImu._accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);


  myImu.Init();
  Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte d = myImu.ReadByte(myImu.AK8963_ADDRESS, myImu.AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
  
  // Get magnetometer calibration from AK8963 ROM
  myImu.InitAK8963(magCalFactory); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
  
  myImu.MagCal(myImu._magBias, myImu._magScale);
  Serial.println("AK8963 mag biases (mG)"); Serial.println(myImu._magBias[0]); Serial.println(myImu._magBias[1]); Serial.println(myImu._magBias[2]);
  Serial.println("AK8963 mag scale (mG)"); Serial.println(myImu._magScale[0]); Serial.println(myImu._magScale[1]); Serial.println(myImu._magScale[2]);
  delay(2000); // add delay to see results before serial spew of data
   
  if(SerialDebug) {
//  Serial.println("Calibration values: ");
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(myImu._magCalFactory[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(myImu._magCalFactory[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(myImu._magCalFactory[2], 2);
  }
#if 0
  // Reset the MS5637 pressure sensor
  MS5637Reset();
  delay(100);
  Serial.println("MS5637 pressure sensor reset...");
  // Read PROM data from MS5637 pressure sensor
  MS5637PromRead(Pcal);
  Serial.println("PROM dta read:");
  Serial.print("C0 = "); Serial.println(Pcal[0]);
  unsigned char refCRC = Pcal[0] >> 12;
  Serial.print("C1 = "); Serial.println(Pcal[1]);
  Serial.print("C2 = "); Serial.println(Pcal[2]);
  Serial.print("C3 = "); Serial.println(Pcal[3]);
  Serial.print("C4 = "); Serial.println(Pcal[4]);
  Serial.print("C5 = "); Serial.println(Pcal[5]);
  Serial.print("C6 = "); Serial.println(Pcal[6]);
  
  nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
  Serial.print("Checksum = "); Serial.print(nCRC); Serial.print(" , should be "); Serial.println(refCRC);  
#endif
#ifdef LCD
  display.clearDisplay();
  display.setCursor(20,0); display.print("MS5637");
  display.setCursor(0,10); display.print("CRC is "); display.setCursor(50,10); display.print(nCRC);
  display.setCursor(0,20); display.print("Should be "); display.setCursor(50,30); display.print(refCRC);
  display.display();
  delay(1000);  
#endif /* LCD */

  attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for INT pin output of MPU9250

  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
    static uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
    static float pitch, yaw, roll;
    static float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
    static uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
    static uint32_t Now = 0;                         // used to calculate integration interval

    static float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
    static float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
    static float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)

    static int16_t  mpu9250Data[7], ak8963Data[3];

    static float temperature;

  // If intPin goes high, all data registers have new data
    if(newData) {
        newData = false;  // reset newData flag
        myImu.ReadMPU9250Data(mpu9250Data);// INT cleared on any read
        // readAccelData(accelCount);  // Read the x/y/z adc values

        // Now we'll calculate the accleration value into actual g's
        ax = (float)mpu9250Data[0]*myImu._aRes - myImu._accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)mpu9250Data[1]*myImu._aRes - myImu._accelBias[1];
        az = (float)mpu9250Data[2]*myImu._aRes - myImu._accelBias[2];

        //   readGyroData(gyroCount);  // Read the x/y/z adc values

        // Calculate the gyro value into actual degrees per second
        gx = (float)mpu9250Data[4]*myImu._gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)mpu9250Data[5]*myImu._gRes;
        gz = (float)mpu9250Data[6]*myImu._gRes;

        myImu.ReadMagData(ak8963Data);  // Read the x/y/z adc values

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        if(myImu.NewMagData()) {
            mx = (float)ak8963Data[1]*myImu._mRes*myImu._magCalFactory[0] - myImu._magBias[0];  // get actual magnetometer value, this depends on scale being set
            my = (float)ak8963Data[1]*myImu._mRes*myImu._magCalFactory[1] - myImu._magBias[1];
            mz = (float)ak8963Data[2]*myImu._mRes*myImu._magCalFactory[2] - myImu._magBias[2];
            mx *= myImu._magScale[0];
            my *= myImu._magScale[1];
            mz *= myImu._magScale[2];
        }
    }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
  // we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
  // positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
  // function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
  // This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
  // Pass gyro rate as rad/s
#ifdef AHRS
    MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz, myImu.delt_t);
    getEulers();
    //MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
#endif
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 50) { // update LCD once per half-second independent of read rate

    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mx ); 
    Serial.print(" my = "); Serial.print( (int)my ); 
    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
#ifdef AHRS
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
#endif
    }               
    // Gyro chip temperature to degrees centigrade
    temperature = ((float) myImu.ReadTempData()) / 333.87 + 21.0;
   // Print temperature in degrees centigrade
    if(SerialDebug) {
      Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }
#if 0
    D1 = MS5637Read(ADC_D1, OSR);  // get raw pressure value
    D2 = MS5637Read(ADC_D2, OSR);  // get raw temperature value
    dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    OFFSET = Pcal[2]*pow(2, 17) + dT*Pcal[4]/pow(2,6);
    SENS = Pcal[1]*pow(2,16) + dT*Pcal[3]/pow(2,7);
 
    Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
//
// Second order corrections
    if(Temperature > 20) 
    {
      T2 = 5*dT*dT/pow(2, 38); // correction for high temperatures
      OFFSET2 = 0;
      SENS2 = 0;
    }
    if(Temperature < 20)                   // correction for low temperature
    {
      T2      = 3*dT*dT/pow(2, 33); 
      OFFSET2 = 61*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
      SENS2   = 29*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
    } 
    if(Temperature < -15)                      // correction for very low temperature
    {
      OFFSET2 = OFFSET2 + 17*(100*Temperature + 1500)*(100*Temperature + 1500);
      SENS2 = SENS2 + 9*(100*Temperature + 1500)*(100*Temperature + 1500);
    }
 // End of second order corrections
 //
     Temperature = Temperature - T2/100;
     OFFSET = OFFSET - OFFSET2;
     SENS = SENS - SENS2;
 
     Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 15))/100;  // Pressure in mbar or kPa
  
    const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters

    float baroin = Pressure; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    float altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;

    float altitude = 145366.45*(1. - pow((Pressure/1013.25), 0.190284));
   
    if(SerialDebug) {
    Serial.print("Digital temperature value = "); Serial.print( (float)Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Digital temperature value = "); Serial.print(9.*(float) Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Digital pressure value = "); Serial.print((float) Pressure, 2);  Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
    }
#endif
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
 //   yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
 //   pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
 //   roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
 //   pitch *= 180.0f / PI;
 //   yaw   *= 180.0f / PI; 
 //   yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
 //   if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
 //   roll  *= 180.0f / PI;
#ifdef AHRS
    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   += 2.23f; // Declination at Bandwaldalle, Karlsruhe is 2.23 degrees on 2017-06-06
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / PI;
    lin_ax = ax + a31;
    lin_ay = ay + a32;
    lin_az = az - a33;
#endif
    if(SerialDebug) {
#ifdef AHRS
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.println(roll, 2);
#endif
      Serial.print("Grav_x, Grav_y, Grav_z: ");
      Serial.print(-a31*1000, 2);
      Serial.print(", ");
      Serial.print(-a32*1000, 2);
      Serial.print(", ");
      Serial.print(a33*1000, 2);  Serial.println(" mg");
      Serial.print("Lin_ax, Lin_ay, Lin_az: ");
      Serial.print(lin_ax*1000, 2);
      Serial.print(", ");
      Serial.print(lin_ay*1000, 2);
      Serial.print(", ");
      Serial.print(lin_az*1000, 2);  Serial.println(" mg");
      
      Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    } else {
      Serial.print(yaw, 2);
      Serial.print("\t");
      Serial.print(pitch, 2);
      Serial.print("\t");
      Serial.print(roll, 2);
      Serial.print("\t");
      
      //Serial.print((float)sumCount/sum, 2);
      Serial.print("\n");
    }

    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void myinthandler()
{
  newData = true;
}

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
