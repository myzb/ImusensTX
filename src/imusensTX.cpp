/*
 * imusensTX.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: may
 */

#include <Arduino.h>
#include <i2c_t3.h>
#include <SPI.h>

#include "utils.h"
#include "FusionFilter.h"
#include "mpu9250.h"
#include "MetroExt.h"
#include "Stopwatch.h"

#define AHRS
#define I2C_SPI_TIME
//#define I2C
//#define RESET_MAGCAL

// Debug flag
// 0: off, 1: std, 2: verbose, 3: vverbose
static const int Debug = 1;

// Pin definitions
static const int intPin1_vhcl = 36; // MPU9250 1 vhcl intPin
static const int intPin2_head = 2;  // MPU9250 2 head intPin
static const int ledPin = 13;

#ifdef I2C_SPI_TIME
// Times the avg sensor readout time
volatile static int i = 0;
volatile static float dt = 0, ts = 0, ts2 = 0;
#endif

// Globals
//mpu9250 vhclImu(0x69, 0, I2C_PINS_33_34);   // MPU9250 1 on I2C bus 0 at addr 0x68
mpu9250 headImu(0x69, 0, I2C_PINS_18_19);   // MPU9250 2 On I2C bus 0 at addr 0x69

mpu9250 vhclImu(35, MOSI_PIN_28, IRS_TRUE);   // MPU9250 1 On SPI bus 0, read called from IRS
//mpu9250 headImu(3,  MOSI_PIN_21, IRS_TRUE);   // MPU9250 2 On SPI bus 0, read called from IRS

FusionFilter vhclFilter;
FusionFilter headFilter;

MetroExt task_filter = MetroExt(100);       // 100 usec
MetroExt task_usbTx  = MetroExt(1000);      //   1 msec
MetroExt task_dbgOut = MetroExt(5000000);   //   5 sec

Stopwatch chrono_1, chrono_2;

static float imuData1[10], imuData2[10];

volatile int flag = 0;

void irs1Func_vhcl()
{
    vhclImu.GetAllData(imuData1, HS_TRUE);
}

void irs2Func_head()
{
#ifdef I2C_SPI_TIME
    ts2 = micros();
#endif /* I2C_SPI_TIME */

    //headImu.GetAllData(imuData2, HS_FALSE);
    headImu.RequestAllData();
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
        // Toggle the LED (signal sensor rx is active)
        digitalWrite(ledPin, !digitalRead(ledPin));
    }
#endif /* I2C_SPI_TIME */
    flag = 1;
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

    // Setup the LED
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);

    if (Debug) Serial.println("Starting ...");

    /* IMU 1 (Vehicle) Setup */
    // Setup bus specifics for this device
    vhclImu.WireSetup();

    // Start the bus (only one device needs to start it on a shared bus)
    vhclImu.WireBegin();

    // Loop forever if MPU9250 is not online
    if (vhclImu.whoAmI() != 0x71) goto next;

    if (Debug) {
        Serial.println("MPU9250 (1): 9-axis motion sensor is online");
    }

    // Start by performing self test and reporting values
    vhclImu.SelfTest(stPercent);
    delay(1000);

    if (Debug) Serial.println("MPU9250 (1): Calibrating gyro and accel");

    // Calibrate gyro and accelerometers, load biases in bias registers
    vhclImu.AcelGyroCal(gyroBias, accelBias);

    if (Debug) Serial.println("MPU9250 (1): Initialising for active data mode....");

    // Config for normal operation
    vhclImu.Init(ACCEL_RANGE_2G, GYRO_RANGE_500DPS, 0x03);    // sample-rate div by 2 (0x01 + 1)

    // Setup interrupts
    vhclImu.SetupInterrupt(intPin1_vhcl, irs1Func_vhcl);

    if (Debug) {
        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        Serial.print("AK8963  (1): I'm "); Serial.println(vhclImu.whoAmIAK8963());
        delay(100);
    }

    // Get magnetometer calibration from AK8963 ROM
    vhclImu.InitAK8963(MAG_RANGE_16BIT, MAG_RATE_100HZ, magCalFactory);

#ifdef RESET_MAGCAL
    // TODO: Add RawHid msg to wave device and also notice on done
    Serial.println("AK8963  (1) initialized for active data mode....");
    Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    delay(4000);
    vhclImu.MagCal(magHardIron, magSoftIron);
#else
    if (Debug) Serial.println("AK8963  (1): Mag calibration using pre-recorded values");
    vhclImu.SetMagCal(magHardIron, magSoftIron);
#endif /* RESET_MAGCAL */

next:
    /* IMU 2 (Head) Setup */
    // Setup bus specifics for this device
    headImu.WireSetup();

    // Start the bus (only one device needs to start it on a shared bus)
    headImu.WireBegin();

    // Loop forever if MPU9250 is not online
    if (headImu.whoAmI() != 0x71) goto end;

    if (Debug) Serial.println("MPU9250 (2): 9-axis motion sensor is online");

    // Start by performing self test and reporting values
    headImu.SelfTest(stPercent);
    delay(1000);

    if (Debug) Serial.println("MPU9250 (2): Calibrating gyro and accel");

    // Calibrate gyro and accelerometers, load biases in bias registers
    headImu.AcelGyroCal(gyroBias, accelBias);

    if (Debug) Serial.println("MPU9250 (2): Initialising for active data mode....");

    // Config for normal operation
    headImu.Init(ACCEL_RANGE_2G, GYRO_RANGE_500DPS, 0x03);    // sample-rate div by 2 (0x01 + 1)

    // Setup interrupts
    headImu.SetupInterrupt(intPin2_head, irs2Func_head);

    if (Debug) {
        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        Serial.print("AK8963  (2): I'm "); Serial.println(headImu.whoAmIAK8963());
        delay(100);
    }

    // Get magnetometer calibration from AK8963 ROM
    headImu.InitAK8963(MAG_RANGE_16BIT, MAG_RATE_100HZ, magCalFactory);

#ifdef RESET_MAGCAL
    // TODO: Add RawHid msg to wave device and also notice on done
    Serial.println("AK8963  (2) initialized for active data mode....");
    Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    delay(4000);
    headImu.MagCal(magHardIron, magSoftIron);
#else
    if (Debug) Serial.println("AK8963  (2): Mag calibration using pre-recorded values");
    headImu.SetMagCal(magHardIron, magSoftIron);
#endif /* RESET_MAGCAL */

end:
    // Enable interrupts
    vhclImu.EnableInterrupt();
    headImu.EnableInterrupt();

    // Wait for the host application to be ready
    if (Debug) Serial.println("Setup done!");
    while (!RawHID.available());

    chrono_1.Reset();
    chrono_2.Reset();
}

void loop()
{
    static uint32_t filterCnt = 0;                      // loop counter

    static data_t rx_buffer, tx_buffer = { 0 };         // usb rx/tx buffer zero initialized
    static uint8_t num;                                 // usb return code
    static uint32_t lastTx = 0, lastRx;                 // last rx/tx time of usb data
    static uint32_t packetCount = 0;                    // usb packet number

    noInterrupts();

    // Read the DMA buffer after the request flag has been set and once i2c is done
    if (headImu.RequestedAvailable()) {
        headImu.GetAllData(imuData2, HS_FALSE);
    }

#ifdef AHRS
    if (task_filter.check()) {

        vhclFilter.MadgwickUpdate(imuData1[0], imuData1[1], imuData1[2],
                                 imuData1[4], imuData1[5], imuData1[6],
                                 imuData1[7], imuData1[8], imuData1[9], chrono_1.Split());

        headFilter.MadgwickUpdate(imuData2[0], imuData2[1], imuData2[2],
                                 imuData2[4], imuData2[5], imuData2[6],
                                 imuData2[7], imuData2[8], imuData2[9], chrono_2.Split());


        // Get quat rotation difference, store result in tx_buffer[0:3]
        quatDiv(vhclFilter.GetQuat(), headFilter.GetQuat(), tx_buffer.num_f);

        filterCnt++;
    }
#endif /* AHRS */

#if 0
    Serial.print(q1[0],2); Serial.print(" ");   Serial.print(q1[1],2); Serial.print(" ");
    Serial.print(q1[2],2); Serial.print(" "); Serial.println(q1[3],2);

    Serial.print(q2[0],2); Serial.print(" ");   Serial.print(q2[1],2); Serial.print(" ");
    Serial.print(q2[2],2); Serial.print(" "); Serial.println(q2[3],2);

    Serial.print(q[0],2); Serial.print(" ");   Serial.print(q[1],2); Serial.print(" ");
    Serial.print(q[2],2); Serial.print(" "); Serial.println(q[3],2); Serial.print("\n");
#endif

    interrupts();

    // Set usb transfer rate to 1kHz TODO: User a timer
    if (micros() - lastTx >= 1000) {

        // Place packetCount into last 4 bytes
        tx_buffer.num_d[15] = packetCount;

        // Send the packet
        noInterrupts();
        num = RawHID.send(tx_buffer.raw, 10);
        interrupts();
        if (num > 0) {
            lastTx = micros();
            packetCount = packetCount + 1;

        } else if (Debug == 2) {
            Serial.print(F("TX: Fail \t")); Serial.println(num);
        }
    }

    // TODO: Use a timer to query each X millis
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

#if 1
    /* Major debug print routines */
    // time for 1 loop = loopCountTime/filterCnt. this is the filter update rate

    // Print current vals each 5sec
    if (task_dbgOut.check()) {

        if (Debug) {
            // Print the avg loop rate
            Serial.print("filter: rate = "); Serial.print((float)filterCnt/5.0f, 2);
            Serial.println(" Hz");
        }
        if (1) {
            Serial.print(imuData1[0], 4); Serial.print(" ");
            Serial.print(imuData1[1], 4); Serial.print(" "); Serial.println(imuData1[2], 4);
            Serial.println(imuData1[3],4);
            Serial.print(imuData1[4], 4); Serial.print(" ");
            Serial.print(imuData1[5], 4); Serial.print(" "); Serial.println(imuData1[6], 4);
            Serial.print(imuData1[7], 4); Serial.print(" ");
            Serial.print(imuData1[8], 4); Serial.print(" "); Serial.println(imuData1[9], 4);
            Serial.print("\n");

            Serial.print(imuData2[0], 4); Serial.print(" ");
            Serial.print(imuData2[1], 4); Serial.print(" "); Serial.println(imuData2[2], 4);
            Serial.println(imuData2[3],4);
            Serial.print(imuData2[4], 4); Serial.print(" ");
            Serial.print(imuData2[5], 4); Serial.print(" "); Serial.println(imuData2[6], 4);
            Serial.print(imuData2[7], 4); Serial.print(" ");
            Serial.print(imuData2[8], 4); Serial.print(" "); Serial.println(imuData2[9], 4);
            Serial.print("\n");
        }

        // Toggle the LED (signal mainloop is running)
        //digitalWrite(ledPin, !digitalRead(ledPin));

        // Reset print counters
        filterCnt = 0;
    }
#endif
}
