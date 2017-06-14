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
#include "Arduino.h"


#include "utils.h"
#include "quaternionFilters.h"
#include "mpu9250.h"

#define SERIAL_EXPORT

// Pin definitions
static const int intPin = 33;
static const int myLed = 13;

// Debug configs
static const bool SerialDebug = true; // FIXME: reorganize/clean all debug prints

// Globals
volatile bool newData = false;
mpu9250 myImu;

void intFunc()
{
  newData = true;
}

void setup()
{
    float gyroBias[3]; //, accelBias; // FIXME: Move mpu9250::_accelBias here
    float selfTest[6];
    float magCalFactory[3];

    // Wire.begin();
    // TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
    // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    delay(4000);
    Serial.begin(38400);
  
    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    if (SerialDebug) {
        I2Cscan();// look for I2C devices on the bus
        // Read the WHO_AM_I register, this is a good test of communication
        Serial.println("MPU9250 9-axis motion sensor...");
    }

    byte c = myImu.ReadByte(myImu.MPU9250_ADDRESS, myImu.WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

    // WHO_AM_I should always be 0x68
    if (c == 0x71) {
        if (SerialDebug)
            Serial.println("MPU9250 is online...");

        myImu.SelfTest(selfTest); // Start by performing self test and reporting values
        delay(1000);

        if (SerialDebug) {
            Serial.print("x-axis self test: acceleration trim within : "); Serial.print(selfTest[0],1); Serial.println("% of factory value");
            Serial.print("y-axis self test: acceleration trim within : "); Serial.print(selfTest[1],1); Serial.println("% of factory value");
            Serial.print("z-axis self test: acceleration trim within : "); Serial.print(selfTest[2],1); Serial.println("% of factory value");
            Serial.print("x-axis self test: gyration trim within : "); Serial.print(selfTest[3],1); Serial.println("% of factory value");
            Serial.print("y-axis self test: gyration trim within : "); Serial.print(selfTest[4],1); Serial.println("% of factory value");
            Serial.print("z-axis self test: gyration trim within : "); Serial.print(selfTest[5],1); Serial.println("% of factory value");
        }

        // set sensor resolutions, only need to do this once
        myImu.SetAres(mpu9250::AFS_2G);
        myImu.SetGres(mpu9250::GFS_250DPS);
        myImu.SetMres(mpu9250::MFS_16BITS);
        myImu.SetMmode(mpu9250::MRATE_100HZ);

        if (SerialDebug)
            Serial.println("Calibrate gyro and accel");

        myImu.AcelGyroCal(gyroBias, myImu._accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

        if (SerialDebug) {
            Serial.println("accel biases (mg)"); Serial.println(1000.*myImu._accelBias[0]); Serial.println(1000.*myImu._accelBias[1]); Serial.println(1000.*myImu._accelBias[2]);
            Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
            Serial.println("MPU9250 initialized for active data mode....");
        }

        myImu.Init();


        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        byte d = myImu.ReadByte(myImu.AK8963_ADDRESS, myImu.AK8963_WHO_AM_I);

        if(SerialDebug)
            Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

        // Get magnetometer calibration from AK8963 ROM
        myImu.InitAK8963(magCalFactory);

        if (SerialDebug) {
            Serial.println("AK8963 initialized for active data mode....");
            Serial.println("Mag Calibration: Wave device in a figure eight until done!");
            delay(4000);
        }

        // FIXME: Add RawHid msg to wave device and also notice on done
        myImu.MagCal(myImu._magBias, myImu._magScale);

        if (SerialDebug) {
            Serial.println("Mag Calibration done!");
            Serial.println("AK8963 mag biases (mG)"); Serial.println(myImu._magBias[0]); Serial.println(myImu._magBias[1]); Serial.println(myImu._magBias[2]);
            Serial.println("AK8963 mag scale (mG)"); Serial.println(myImu._magScale[0]); Serial.println(myImu._magScale[1]); Serial.println(myImu._magScale[2]);

            delay(2000);

            Serial.println("Calibration values: ");
            Serial.print("X-Axis sensitivity adjustment value "); Serial.println(myImu._magCalFactory[0], 2);
            Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(myImu._magCalFactory[1], 2);
            Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(myImu._magCalFactory[2], 2);
        }

        // Attach interrupt pin to MPU9250 and define interrupt function
        attachInterrupt(intPin, intFunc, RISING);
    } else {
        Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
        Serial.print(" I should be "); Serial.println(0x71, HEX);
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);
        while(1) ; // Loop forever if communication doesn't happen
    }
}

void loop()
{
    static uint32_t loopCount = 0;                      // loop counter
    static float loopCountTime = 0;                     // loop time counter
    static uint32_t deltaPrint = 0, printTime = 0;      // time of serial debug and delta to last print

    static uint32_t now, lastUpdate = 0;                // used to calculate integration interval
    static float deltat = 0.0f;                         // time (integration interval) between filter updates

    static float ax, ay, az, gx, gy, gz, mx, my, mz;    // variables to hold latest data values
    static int16_t  mpu9250Data[7], ak8963Data[3];      // raw accel/gyro & mag data

    static byte buffer[64];                             // usb rx/tx buffer
    static uint8_t n;                                   // return code vars
    static uint32_t lastTx = 0;                         // Last time usb data was send

    // If intPin goes high, all data registers have new data
    if(newData) {
        newData = false;  // reset newData flag
        myImu.ReadMPU9250Data(mpu9250Data);// INT cleared on any read

        // readAccelData(accelCount);  // Read the x/y/z adc values

        // Now we'll calculate the accleration value into actual g's
        // get actual g value, this depends on scale being set
        ax = (float)mpu9250Data[0]*myImu._aRes - myImu._accelBias[0];
        ay = (float)mpu9250Data[1]*myImu._aRes - myImu._accelBias[1];
        az = (float)mpu9250Data[2]*myImu._aRes - myImu._accelBias[2];

        //   readGyroData(gyroCount);  // Read the x/y/z adc values

        // Calculate the gyro value into actual degrees per second
        // get actual gyro value, this depends on scale being set
        gx = (float)mpu9250Data[4]*myImu._gRes;
        gy = (float)mpu9250Data[5]*myImu._gRes;
        gz = (float)mpu9250Data[6]*myImu._gRes;

        myImu.ReadMagData(ak8963Data);  // Read the x/y/z adc values

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        if(myImu.NewMagData()) {
            // get magnetometer value, this depends on scale being set
            mx = (float)ak8963Data[0]*myImu._mRes*myImu._magCalFactory[0] - myImu._magBias[0];
            my = (float)ak8963Data[1]*myImu._mRes*myImu._magCalFactory[1] - myImu._magBias[1];
            mz = (float)ak8963Data[2]*myImu._mRes*myImu._magCalFactory[2] - myImu._magBias[2];
            mx *= myImu._magScale[0];
            my *= myImu._magScale[1];
            mz *= myImu._magScale[2];
#ifdef SERIAL_EXPORT
            Serial.print( (int)((float)ak8963Data[0]*myImu._mRes) ); Serial.print("\t");
            Serial.print( (int)((float)ak8963Data[1]*myImu._mRes) ); Serial.print("\t");
            Serial.print( (int)((float)ak8963Data[2]*myImu._mRes) ); Serial.print("\t");
            Serial.print( (int) mx ); Serial.print("\t");
            Serial.print( (int) my ); Serial.print("\t");
            Serial.print( (int) mz ); Serial.print("\n");
#endif /* EXPORT_VALS */
        }
    }

#ifndef SERIAL_EXPORT
    if (micros() - lastTx > 1000) {
        lastTx = micros();
        static int packetCount = 0;
        data_t reading = {
                -ax, ay, az,
                gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,
                my, -mx, mz, deltat
        };

        // next 24 bytes are analog measurements
        for (int i=0; i<40; i++) {
            buffer[i] = reading.raw[i];
        }

        // fill the rest with zeros
        for (int i=40; i<62; i++) {
            buffer[i] = 0;
        }
        // and put a count of packets sent at the end
        buffer[62] = highByte(packetCount);
        buffer[63] = lowByte(packetCount);
        // actually send the packet
        n = RawHID.send(buffer, 1);
        if (n > 0) {
            if (SerialDebug) {
                Serial.print(F("Transmit packet "));
                Serial.println(packetCount);
            }
             packetCount = packetCount + 1;
        } else if (SerialDebug) {
            Serial.println(F("Unable to transmit packet"));
        }
    }
#endif /* SERIAL_EXPORT */

    now = micros();
    deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = now;
    //deltat =  getTimeDelta(&lastUpdate);

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
    MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, my, -mx, mz, deltat);
    //MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
#endif
    // Serial print and/or display at 0.5 s rate independent of data rates

    // time for 1 loop = loopCountTime/loopCount
    loopCountTime += deltat;
    loopCount++;

    // Print current vals each 500ms
    deltaPrint = millis() - printTime;
    if (deltaPrint > 500) {

#ifndef SERIAL_EXPORT
        if (SerialDebug) {
            //dumpData(ax, ay, az, gx, gy, gz, mx, my, mz, myImu.ReadTempData()); // FIXME: inline version is faster

            // Print the avg loop rate
            Serial.print("rate = "); Serial.print((float)loopCount/loopCountTime, 2); Serial.println(" Hz");
        }
#endif /* SERIAL_EXPORT */

        // Toggle the LED
        digitalWrite(myLed, !digitalRead(myLed));

        // Reset print counters
        printTime = millis();
        loopCount = 0;
        loopCountTime = 0;
    }
}
