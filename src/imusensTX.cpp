/*
 * imusensTX.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: may
 */

#include <i2c_t3.h>
#include <SPI.h>

#include "Arduino.h"

#include "utils.h"
#include "quaternionFilters.h"
#include "mpu9250.h"

#define AHRS
#define I2C_SPI_TIME
//#define RESET_MAGCAL

// Debug flag
// 0: off, 1: std, 2: verbose, 3: vverbose
static const int Debug = 1;

// Pin definitions
//static const int intPin = 33; // MPU9250 1 intPin
static const int intPin = 16;   // MPU9250 2 intPin
static const int myLed = 13;

#ifdef I2C_SPI_TIME
// Times the avg sensor readout time
volatile static int i = 0;
volatile static float dt = 0, ts = 0, ts2 = 0;
#endif

// Globals
//mpu9250 myImu(34, MOSI_PIN_28);   // MPU9250 1 On SPI bus 0
mpu9250 myImu(17, MOSI_PIN_28);     // MPU9250 2 On SPI bus 0
static float imuData[10];

void intFunc()
{
#ifdef I2C_SPI_TIME
    ts2 = micros();
#endif /* I2C_SPI_TIME */
    myImu.GetAllData(false, imuData);
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

        // Toggle the LED
        digitalWrite(myLed, !digitalRead(myLed));
    }
#endif /* I2C_SPI_TIME */
}

void setup()
{
    float gyroBias[3], accelBias[3];
    float stPercent[6];
    float magCalFactory[3];

#if 1
    // Pre-calibrated values (office)
    float magHardIron[3] = {6.21309566f, 48.00038147f, -26.88592911};
    float magSoftIron[3] = {1.05979073f, 0.90549165f, 1.05037034f};
#else
    // Pre-calibrated values (golf)
    float magHardIron[3] = {47.51190567f, 584.83221436f, -210.48852539};
    float magSoftIron[3] = {1.02854228f, 0.99213374f, 0.98056370f};
#endif

    Serial.begin(38400);
    delay(4000);

    // Init I2C/SPI for this sensor
    myImu.WireBegin();

    Serial.println("Starting ...");

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    // Loop forever if MPU9250 is not online
    if (myImu.whoAmI() != 0x71) while(1);

    if (Debug) {
        Serial.println("MPU9250: 9-axis motion sensor is online");
    }

    // Start by performing self test and reporting values
    myImu.SelfTest(stPercent);
    delay(1000);

    if (Debug) Serial.println("MPU9250: Calibrating gyro and accel");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myImu.AcelGyroCal(gyroBias, accelBias);

    if (Debug) Serial.println("MPU9250: Initialising for active data mode....");

    // Config for normal operation
    myImu.Init(ACCEL_RANGE_2G, GYRO_RANGE_500DPS, 0x00);    // sample-rate div by 1 (0x00 + 1)

    // Setup interrupts
    myImu.SetupInterrupt();

    if (Debug) {
        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        Serial.print("AK8963: I'm "); Serial.println(myImu.whoAmIAK8963());
        delay(100);
    }

    // Get magnetometer calibration from AK8963 ROM
    myImu.InitAK8963(MAG_RANGE_16BIT, MAG_RATE_100HZ, magCalFactory);

#ifdef RESET_MAGCAL
    // TODO: Add RawHid msg to wave device and also notice on done
    Serial.println("AK8963 initialized for active data mode....");
    Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    delay(4000);
    myImu.MagCal(magHardIron, magSoftIron);
#else
    Serial.println("AK8963: Mag calibration using pre-recorded values");
    myImu.SetMagCal(magHardIron, magSoftIron);
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

    // Calculate loop time
    now = micros();
    deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last data arrival
    lastUpdate = now;

#ifdef AHRS
    MadgwickQuaternionUpdate(imuData[0], imuData[1], imuData[2],
                             imuData[4], imuData[5], imuData[6],
                             imuData[7], imuData[8], imuData[9], deltat);
#endif /* AHRS */

#ifndef NO_USB
    // Set usb transfer rate to 1kHz // TODO: Use a timer
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
            imuData[0], imuData[1], imuData[2],
            imuData[4], imuData[5], imuData[6],
            imuData[7], imuData[8], imuData[9], deltat
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

        if (Debug) {
            // Print the avg loop rate
            Serial.print("loop: rate = "); Serial.print((float)loopCount/loopCountTime, 2);
            Serial.println(" Hz");
        }
        if (Debug) {
            Serial.print(imuData[0], 4); Serial.print(" ");
            Serial.print(imuData[1], 4); Serial.print(" "); Serial.println(imuData[2], 4);
            Serial.println(imuData[3],4);
            Serial.print(imuData[4], 4); Serial.print(" ");
            Serial.print(imuData[5], 4); Serial.print(" "); Serial.println(imuData[6], 4);
            Serial.print(imuData[7], 4); Serial.print(" ");
            Serial.print(imuData[8], 4); Serial.print(" "); Serial.println(imuData[9], 4);
            Serial.print("\n");
        }

        // Toggle the LED
        //digitalWrite(myLed, !digitalRead(myLed));

        // Reset print counters
        lastPrint = millis();
        loopCount = 0;
        loopCountTime = 0;
    }
}
