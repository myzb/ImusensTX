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

#define AHRS
#define I2C_SLV0
#define I2C_SPI_TIME
//#define RESET_MAGCAL

// Debug flag
// 0: off, 1: std, 2: verbose, 3: vverbose
static const int Debug = 1;

// Pin definitions
static const int intPin = 33;
static const int myLed = 13;

#ifdef I2C_SPI_TIME
// Times the avg sensor readout time
volatile static int i = 0;
volatile static float dt = 0, ts = 0, ts2 = 0;
#endif

// Globals
mpu9250 myImu(34, MOSI_PIN_28); // MPU9250 On SPI bus 0
static float imuData[10];

void intFunc()
{
#ifdef I2C_SLV0
#ifdef I2C_SPI_TIME
    ts2 = micros();
#endif /* I2C_SPI_TIME */
    myImu.GetAllData(imuData);
#ifdef I2C_SPI_TIME
    dt = dt + micros() - ts2;
    i++;
    if (i > 1000) {
        ts = micros() - ts;
        Serial.print("IMU: fs = "); Serial.print((float)i/ts *1000000.f); Serial.println(" Hz");
        ts = micros();
        dt = dt/i;
        Serial.print("IMU: I2C/SPI rate = "); Serial.print(dt, 2); Serial.println(" us");
        dt = 0;
        i = 0;
    }
#endif /* I2C_SPI_TIME */
#else
    myImu._newData = 1;
#endif /* I2C_SLV0 */
}

void setup()
{
    float gyroBias[3], accelBias[3];
    float selfTest[6];
    float magCalFactory[3];

#if 1
    // Pre-calibrated values (office)
    float magBias[3] = {21.92857170f, 529.65936279f, -226.40782166f};
    float magScale[3] = {1.04433501f, 0.97695851f, 0.98148149f};
#else
    // Pre-calibrated values (golf)
    float magBias[3] = {47.51190567f, 584.83221436f, -210.48852539};
    float magScale[3] = {1.02854228f, 0.99213374f, 0.98056370f};
#endif

    Serial.begin(38400);
    delay(4000);

    // Init I2C/SPI for this sensor
    myImu.WireBegin();

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

#ifndef I2C_SLV0
    I2Cscan();// look for I2C devices on the bus
#endif /* I2C_SLV0 */

    // Loop forever if MPU9250 is not online
    if (myImu.whoAmI() != 0x71) while(1);

    if (Debug) {
        Serial.println("MPU9250: 9-axis motion sensor is online");
    }

    // Start by performing self test and reporting values
    //myImu.SelfTest(selfTest);
    delay(1000);

    if (Debug) Serial.println("MPU9250: Calibrating gyro and accel");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myImu.AcelGyroCal(gyroBias, accelBias);

    if (Debug) Serial.println("MPU9250: Initialising for active data mode....");

    // Config for normal operation
    myImu.Init(ACCEL_RANGE_2G, GYRO_RANGE_500DPS, 0x04);    // sample-rate div by 4 (0x03 + 1)

    // Setup interrupts
    myImu.SetupInterrupt();

#ifdef I2C_SLV0
    if(Debug) {
        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        Serial.print("AK8963: I'm "); Serial.println(myImu.whoAmIAK8963());
        delay(100);
    }
#endif /* I2C_SLV0 */

    // Get magnetometer calibration from AK8963 ROM
    myImu.InitAK8963(MAG_RANGE_16BIT, MAG_RATE_100HZ, magCalFactory);

#ifdef RESET_MAGCAL
    // TODO: Add RawHid msg to wave device and also notice on done
    myImu.MagCal(myImu._mCal_bias, myImu._mCal_scale);
#else
    Serial.println("AK8963: Mag calibration using pre-recorded values");
    myImu.SetMagCal(magBias, magScale);
#endif /* RESET_MAGCAL */

    // Attach interrupt pin and int function
    attachInterrupt(intPin, intFunc, RISING);
    myImu.EnableInterrupt();

    // Wait for the application to be ready
    Serial.println("Setup done!");
    while (!RawHID.available());
}

void loop()
{
    static uint32_t loopCount = 0;                      // loop counter
    static float loopCountTime = 0;                     // loop time counter
    static uint32_t lastPrint = 0;                      // time of serial debug and delta to last print

    static uint32_t now, lastUpdate = 0;                // used to calculate integration interval
    static float deltat = 0.0f;                         // time (integration interval) between filter updates

    static data_t rx_buffer, tx_buffer;                 // usb rx/tx buffer
    static uint8_t num;                                 // usb return code
    static uint32_t lastTx = 0, lastRx;                 // last rx/tx time of usb data
    static uint32_t packetCount = 0;                    // usb packet number

#ifndef I2C_SLV0
    // If intPin goes high, all data registers have new data
    if(myImu.NewData()) {

#ifdef I2C_SPI_TIME
        ts2 = micros();
#endif /* I2C_SPI_TIME */

        myImu.GetMPU9250Data(imuData);

#ifdef I2C_SPI_TIME
        dt = dt + micros() - ts2;
        i++;
        if (i > 1000) {
            ts = micros() - ts;
            Serial.print("IMU: fs = "); Serial.print((float)i/ts *1000000.f); Serial.println(" Hz");
            ts = micros();
            dt = dt/i;
            Serial.print("IMU: I2C rate = "); Serial.print(dt, 2); Serial.println(" us");
            dt = 0;
            i = 0;
        }
#endif /* I2C_SPI_TIME */

        if(myImu.NewMagData()) {
            myImu.GetMagData(imuData+7); // MagData: imuData[7] ... imuData[10]

            if (Debug == 3) {
                Serial.print(imuData[0], 4);Serial.print(" ");Serial.print(imuData[1], 4);Serial.print(" ");Serial.println(imuData[2], 4);
                Serial.println(imuData[3],4);
                Serial.print(imuData[4], 4);Serial.print(" ");Serial.print(imuData[5], 4);Serial.print(" ");Serial.println(imuData[6], 4);
                Serial.print(imuData[7], 4);Serial.print(" ");Serial.print(imuData[8], 4);Serial.print(" ");Serial.println(imuData[9], 4);
                Serial.print("\n");
            }
        }
    }
#endif /* I2C_SLV0 */

    // Calculate loop time
    now = micros();
    deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last data arrival
    lastUpdate = now;

#ifdef AHRS
    MadgwickQuaternionUpdate(-imuData[0], imuData[1], imuData[2],
                              imuData[4], -imuData[5], -imuData[6],
                              imuData[8], -imuData[7], imuData[9], deltat);
#endif /* AHRS */

#ifndef NO_USB
    // Set usb transfer rate to 1kHz
    if (micros() - lastTx >= 1000) {
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
        num = RawHID.send(tx_buffer.raw, 10);
        if (num > 0) {
            lastTx = micros();
            packetCount = packetCount + 1;

        } else if (Debug == 2) {
            Serial.print(F("TX: Fail \t")); Serial.println(num);

        }
    }

    if (RawHID.available()) {
        // Only send data to device if really necessary as it slows down usb tx
        num = RawHID.recv(rx_buffer.raw, 10);
        if (num > 0) {
            lastRx = micros();

            if (Debug == 2) {
                Serial.print("TX: \t"); Serial.print(tx_buffer.num_d[15]);
                Serial.print("\tRX: \t"); Serial.println(rx_buffer.num_d[15]);
                Serial.print(lastTx / 1000 % 1000, 8);Serial.print("\t\t"); Serial.println(lastRx / 1000 % 1000, 8);
            }

        } else if (Debug == 2) {
            Serial.print(F("\tRX: Fail \t")); Serial.println(num);
        }
    }
#endif /* NO_USB */

    // time for 1 loop = loopCountTime/loopCount. this is the filter update rate
    loopCountTime += deltat;
    loopCount++;

    // Print current vals each 2000ms
    if (millis() - lastPrint >= 2000) {

        // TODO: Inline version is faster
        //dumpData(ax, ay, az, gx, gy, gz, mx, my, mz, myImu.ReadTempData());
        if(Debug) {
            // Print the avg loop rate
            Serial.print("loop: rate = "); Serial.print((float)loopCount/loopCountTime, 2);
            Serial.println(" Hz");
        }

        // Toggle the LED
        digitalWrite(myLed, !digitalRead(myLed));

        // Reset print counters
        lastPrint = millis();
        loopCount = 0;
        loopCountTime = 0;
    }
}
