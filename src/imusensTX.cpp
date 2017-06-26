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

//#define AHRS

// Pin definitions
static const int intPin = 33;
static const int myLed = 13;

// FIXME: reorganize/clean all debug prints
static const bool vDebug = true;
static const bool Debug = true;

// Globals
volatile bool newData = false;
mpu9250 myImu;

void intFunc()
{
  newData = true;
}

void setup()
{
    float gyroBias[3], accelBias[3];
    float selfTest[6];
    float magCalFactory[3];

    float magBias[3] = {21.92857170f, 529.65936279f, -226.40782166f};   // Pre-calibrated values
    float magScale[3] = {1.04433501f, 0.97695851f, 0.98148149f};

    // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.6
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    delay(4000);
    Serial.begin(38400);
  
    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    if (Debug) {
        I2Cscan();// look for I2C devices on the bus
        // Read the WHO_AM_I register, this is a good test of communication
        Serial.println("MPU9250 9-axis motion sensor...");
    }

    byte c = myImu.ReadByte(myImu.MPU9250_ADDRESS, myImu.WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

    // WHO_AM_I should always be 0x68
    if (c == 0x71) {
        if (Debug)
            Serial.println("MPU9250 is online...");

        myImu.SelfTest(selfTest); // Start by performing self test and reporting values
        delay(1000);

        if (Debug) {
            Serial.print("x-axis self test: acceleration trim within : "); Serial.print(selfTest[0],1); Serial.println("% of factory value");
            Serial.print("y-axis self test: acceleration trim within : "); Serial.print(selfTest[1],1); Serial.println("% of factory value");
            Serial.print("z-axis self test: acceleration trim within : "); Serial.print(selfTest[2],1); Serial.println("% of factory value");
            Serial.print("x-axis self test: gyration trim within : "); Serial.print(selfTest[3],1); Serial.println("% of factory value");
            Serial.print("y-axis self test: gyration trim within : "); Serial.print(selfTest[4],1); Serial.println("% of factory value");
            Serial.print("z-axis self test: gyration trim within : "); Serial.print(selfTest[5],1); Serial.println("% of factory value");
        }

        // set sensor resolutions, only need to do this once
        myImu.SetAres(mpu9250::AFS_2G);
        myImu.SetGres(mpu9250::GFS_500DPS);
        myImu.SetMres(mpu9250::MFS_16BITS);
        myImu.SetMrate(mpu9250::MRATE_100HZ);

        if (Debug)
            Serial.println("Calibrate gyro and accel");

        myImu.AcelGyroCal(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

        if (Debug) {
            Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
            Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
            Serial.println("MPU9250 initialized for active data mode....");
        }

        myImu.Init();

        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        byte d = myImu.ReadByte(myImu.AK8963_ADDRESS, myImu.AK8963_WHO_AM_I);

        if(Debug)
            Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

        // Get magnetometer calibration from AK8963 ROM
        myImu.InitAK8963(magCalFactory);

//#define RESET_MAGCAL
#ifdef RESET_MAGCAL
        if (Debug) {
            Serial.println("AK8963 initialized for active data mode....");
            Serial.println("Mag Calibration: Wave device in a figure eight until done!");
            delay(4000);
        }

        // FIXME: Add RawHid msg to wave device and also notice on done
        myImu.MagCal(myImu._mCal_bias, myImu._mCal_scale);
#else
        Serial.println("Mag Calibration: Using pre-recorded calibration values");
        myImu.SetMagCal(magBias, magScale);
#endif /* RESET_MAGCAL */

        if (Debug) {
            Serial.println("Mag Calibration done!");
            Serial.println("AK8963 mag biases (mG)"); Serial.println(myImu._mCal_bias[0], 8);
            Serial.println(myImu._mCal_bias[1], 8); Serial.println(myImu._mCal_bias[2], 8);
            Serial.println("AK8963 mag scale (mG)"); Serial.println(myImu._mCal_scale[0], 8);
            Serial.println(myImu._mCal_scale[1], 8); Serial.println(myImu._mCal_scale[2], 8);

            delay(2000);

            Serial.println("Calibration values: ");
            Serial.print("X-Axis sensitivity adjustment value "); Serial.println(myImu._mRes_factory[0], 2);
            Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(myImu._mRes_factory[1], 2);
            Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(myImu._mRes_factory[2], 2);
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

    static data_t rx_buffer, tx_buffer;                 // usb rx/tx buffer
    static uint8_t n;                                   // return code vars
    static uint32_t lastTx = 0, lastRx;                 // Last time usb data was send
    static uint32_t packetCount = 0;                    // usb packet number
    static bool ack = true;

    // If intPin goes high, all data registers have new data
    if(newData) {
        newData = false;  // reset newData flag
        myImu.ReadMPU9250Data(mpu9250Data);// INT cleared on any read

        // readAccelData(accelCount);  // Read the x/y/z adc values

        // Now we'll calculate the accleration value into actual g's
        // get actual g value, this depends on scale being set
#if 0
        ax = (float)mpu9250Data[0]*myImu._aRes - myImu._aCal_bias[0];
        ay = (float)mpu9250Data[1]*myImu._aRes - myImu._aCal_bias[1];
        az = (float)mpu9250Data[2]*myImu._aRes - myImu._aCal_bias[2];
#else
        ax = (float)mpu9250Data[0]*myImu._aRes;
        ay = (float)mpu9250Data[1]*myImu._aRes;
        az = (float)mpu9250Data[2]*myImu._aRes;
#endif

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

#ifndef ONLINE_CAL
            mx = (float)ak8963Data[0]*myImu._mRes*myImu._mRes_factory[0] - myImu._mCal_bias[0];
            my = (float)ak8963Data[1]*myImu._mRes*myImu._mRes_factory[1] - myImu._mCal_bias[1];
            mz = (float)ak8963Data[2]*myImu._mRes*myImu._mRes_factory[2] - myImu._mCal_bias[2];
            mx *= myImu._mCal_scale[0];
            my *= myImu._mCal_scale[1];
            mz *= myImu._mCal_scale[2];
#else
            mx = (float)ak8963Data[0]*myImu._mRes*myImu._mRes_factory[0] - myImu._mCal_bias[0];
            my = (float)ak8963Data[1]*myImu._mRes*myImu._mRes_factory[1] - myImu._mCal_bias[1];
            mz = (float)ak8963Data[2]*myImu._mRes*myImu._mRes_factory[2] - myImu._mCal_bias[2];
            mx *= myImu._mCal_scale[0];
            my *= myImu._mCal_scale[1];
            mz *= myImu._mCal_scale[2];
#endif /* ONLINE_CAL */

//#define MAG_EXPORT
#ifdef MAG_EXPORT
            Serial.print( (int)((float)ak8963Data[0]*myImu._mRes) ); Serial.print("\t");
            Serial.print( (int)((float)ak8963Data[1]*myImu._mRes) ); Serial.print("\t");
            Serial.print( (int)((float)ak8963Data[2]*myImu._mRes) ); Serial.print("\t");
            Serial.print( (int) mx ); Serial.print("\t");
            Serial.print( (int) my ); Serial.print("\t");
            Serial.print( (int) mz ); Serial.print("\n");
#endif /* MAG_EXPORT */

        }
    }


    // Get time of new data arrival
    now = micros();
    deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last data arrival
    lastUpdate = now;
#ifdef AHRS
    MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, my, -mx, mz, deltat);
    //MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz, deltat);
#endif

//#define NO_USB
#define ROUNDTRIP

#ifndef NO_USB
    if (millis() - lastTx > 1 && ack) {

        // Prepare usb packet
#ifdef AHRS
        tx_buffer = {
                *(getQ()), *(getQ()+1), *(getQ()+2), *(getQ()+3),
                /* 48 extra bytes */
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
        };
#else
        tx_buffer = {
                -ax, ay, az,
                gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,
                my, -mx, mz, deltat,
                /* 24 extra bytes */
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
        };
#endif /* AHRS */
        // Place packetCount into last 4 bytes
        tx_buffer.num_d[15] = packetCount;

        // Send the packet
        n = RawHID.send(tx_buffer.raw, 10);
        if (n > 0) {
            ack = false;
            lastTx = millis();
            packetCount = packetCount + 1;

            if (vDebug)
                Serial.print("TX: \t"); Serial.print(tx_buffer.num_d[15]);

        } else if (vDebug) {
            Serial.print(F("TX: Fail \t")); Serial.print(n);
        }
    }

#ifdef ROUNDTRIP
    if (RawHID.available()) {
        n = RawHID.recv(rx_buffer.raw, 10);
        if (n > 0) {
            ack = true;
            lastRx = millis();

            if (vDebug)
            Serial.print("\tRX: \t"); Serial.println(rx_buffer.num_d[15]);

        } else if (vDebug) {
            Serial.print(F("\tRX: Fail \t")); Serial.println(n);
        }
        Serial.print(lastTx, 8);Serial.print("\t\t"); Serial.println(lastRx, 8);
#endif /* ROUNDTRIP */
    }
#endif /* NO_USB */

    // time for 1 loop = loopCountTime/loopCount
    loopCountTime += deltat;
    loopCount++;

    // Print current vals each 2000ms
    deltaPrint = millis() - printTime;
    if (deltaPrint > 2000) {

        // Fixme: inline version is faster
        //dumpData(ax, ay, az, gx, gy, gz, mx, my, mz, myImu.ReadTempData());
        if(Debug) {
            // Print the avg loop rate
            Serial.print("rate = "); Serial.print((float)loopCount/loopCountTime, 2);
            Serial.println(" Hz");
        }

        // Toggle the LED
        digitalWrite(myLed, !digitalRead(myLed));

        // Reset print counters
        printTime = millis();
        loopCount = 0;
        loopCountTime = 0;
    }
}
