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
#include "ComplementaryFilter.h"
#include "mpu9250.h"
#include "MetroExt.h"
#include "Stopwatch.h"

#define AHRS
#define I2C_SPI_TIME

// Debug flag
// 0: off, 1: std, 2: verbose, 3: vverbose
static const int Debug = 2;

// Pin definitions
static const int ledPin = 13;
static const int intPin1_vhcl = 9;  // MPU9250 1 vhcl intPin
static const int intPin2_head = 18; // MPU9250 2 head intPin

#ifdef I2C_SPI_TIME
// Times the avg sensor readout time
volatile static int irsCnt = 0;
volatile static float dt = 0, ts = 0;
#endif

// Globals
mpu9250 vhclMarg(10, MOSI_PIN_28);       // MPU9250 1 On SPI bus 0 at csPin 10
mpu9250 headMarg(17, MOSI_PIN_21);       // MPU9250 2 On SPI bus 1 at csPin 17

#ifdef MADGWICK
FusionFilter vhclFilter, headFilter;
#else
Filter vhclFilter, headFilter;
#endif
Stopwatch chrono_1, chrono_2;

MetroExt task_filter = MetroExt(100);       // 100 usec
MetroExt task_usbTx  = MetroExt(1000);      //   1 msec
MetroExt task_dbgOut = MetroExt(2000000);   //   2 sec

static float margData1[10], margData2[10];

void irs1Func_vhcl()
{
    vhclMarg.GetAllRaw(margData1, HS_TRUE);
}

void irs2Func_head()
{
#ifdef I2C_SPI_TIME
    ts = micros();
#endif /* I2C_SPI_TIME */

    headMarg.GetAllRaw(margData2, HS_TRUE);

#ifdef I2C_SPI_TIME
    dt += micros() - ts;
    irsCnt++;
#endif /* I2C_SPI_TIME */
}

void setup()
{
#if 1
    // Pre-calibrated values (office)
    float magHardIron[3] = {6.21309566f, 48.00038147f, -26.88592911};
    float magSoftIron[3] = {1.05979073f, 0.90549165f, 1.05037034f};
#else
    // Pre-calibrated values (golf)
    float magHardIron[3] = {47.51190567f, 584.83221436f, -210.48852539};
    float magSoftIron[3] = {1.02854228f, 0.99213374f, 0.98056370f};
#endif

    if (Debug) Serial.begin(38400);
    delay(4000);

    // Setup the LED
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);

    noInterrupts();
    if (Debug) Serial.printf("Starting ... \n");

    /* IMU 1 (Vehicle) Setup */
    // Setup bus specifics for this device
    vhclMarg.WireSetup();

    // Start the bus (only one device needs to start it on a shared bus)
    vhclMarg.WireBegin();

    // Skip setup if this MPU9250 is not online
    if (vhclMarg.whoAmI() != 0x71) goto next;

    if (Debug) Serial.printf("MPU9250 (1): 9-axis motion sensor is online\n");

    // Start by performing self test (and reporting values)
    vhclMarg.SelfTest();
    delay(1000);

    if (Debug) Serial.printf("MPU9250 (1): Calibrating gyro and accel\n");

    // Calibrate gyro and accelerometers, load biases into bias registers
    vhclMarg.AcelGyroCal();

    if (Debug) Serial.printf("MPU9250 (1): Initialising for active data mode ...\n");

    // Config for normal operation, set sample-rate div to '0x0X + 1'
    vhclMarg.Init(ACCEL_RANGE_2G, GYRO_RANGE_500DPS, 0x00);

    // Check if AK8963 magnetometer is online
    if (Debug) Serial.printf("AK8963  (1): I'm 0x%02x\n", vhclMarg.whoAmIAK8963());

    // Get magnetometer calibration from AK8963 ROM
    vhclMarg.InitAK8963(MAG_RANGE_16BIT, MAG_RATE_100HZ);

#ifdef RESET_MAGCAL
    // TODO: Add RawHid msg to wave device and also notice on done
    Serial.printf("AK8963  (1) initialized for active data mode ...\n");
    Serial.printf("Mag Calibration: Wave device in a figure eight until done!\n");
    delay(4000);
    vhclMarg.MagCal();
#else
    if (Debug) Serial.printf("AK8963  (1): Mag calibration using pre-recorded values\n");
    vhclMarg.SetMagCal(magHardIron, magSoftIron);
#endif /* RESET_MAGCAL */

next:
    /* IMU 2 (Head) Setup */
    // Setup bus specifics for this device
    headMarg.WireSetup();

    // Start the bus (only one device needs to start it on a shared bus)
    headMarg.WireBegin();

    // Skip setup if this MPU9250 is not online
    if (headMarg.whoAmI() != 0x71) goto end;

    if (Debug) Serial.printf("MPU9250 (2): 9-axis motion sensor is online\n");

    // Start by performing self test (and reporting values)
    headMarg.SelfTest();
    delay(1000);

    if (Debug) Serial.printf("MPU9250 (2): Calibrating gyro and accel\n");

    // Calibrate gyro and accelerometers, load biases into bias registers
    headMarg.AcelGyroCal();

    if (Debug) Serial.printf("MPU9250 (2): Initialising for active data mode...\n");

    // Config for normal operation, set sample-rate div to '0x0X + 1'
    headMarg.Init(ACCEL_RANGE_2G, GYRO_RANGE_500DPS, 0x00);

    // Check if AK8963 magnetometer is online
    if (Debug) Serial.printf("AK8963  (2): I'm 0x%02x\n", headMarg.whoAmIAK8963());

    // Get magnetometer calibration from AK8963 ROM
    headMarg.InitAK8963(MAG_RANGE_16BIT, MAG_RATE_100HZ);

#ifdef RESET_MAGCAL
    // TODO: Add RawHid msg to wave device and also notice on done
    Serial.printf("AK8963  (2) initialized for active data mode...\n");
    Serial.printf("Mag Calibration: Wave device in a figure eight until done!\n");
    delay(4000);
    headMarg.MagCal();
#else
    if (Debug) Serial.printf("AK8963  (2): Mag calibration using pre-recorded values\n");
    headMarg.SetMagCal(magHardIron, magSoftIron);
#endif /* RESET_MAGCAL */

end:
    if (Debug) Serial.printf("\nSetup done, enabling interrupts!\n");

    // Enable interrupts
    vhclMarg.EnableInterrupt(intPin1_vhcl, irs1Func_vhcl);
    headMarg.EnableInterrupt(intPin2_head, irs2Func_head);
    interrupts();

    // Wait for the host application to be ready
    while (!RawHID.available());

    chrono_1.Reset();
    chrono_2.Reset();
}

void loop()
{
    static uint32_t filterCnt = 0;              // loop counter
    static data_t rx_buffer, tx_buffer = { 0 }; // usb rx/tx buffer zero initialized
    static uint8_t num;                         // usb return code
    static uint32_t lastTx = 0, lastRx = 0;     // last rx/tx time of usb data
    static uint32_t pktCnt = 0;                 // usb packet number


#ifdef AHRS
    /* Task 1 - Filter sensor data @ 0.1 msec (10 kHz) */
    if (task_filter.check()) {

        noInterrupts();
#ifdef MADGWICK
        vhclFilter.MadgwickUpdate(margData1[0], margData1[1], margData1[2],
                                  margData1[4], margData1[5], margData1[6],
                                  margData1[7], margData1[8], margData1[9], chrono_1.Split());

        headFilter.MadgwickUpdate(margData2[0], margData2[1], margData2[2],
                                  margData2[4], margData2[5], margData2[6],
                                  margData2[7], margData2[8], margData2[9], chrono_2.Split());
#else
        vhclFilter.Prediction(&margData1[4], chrono_1.Split());
        //headFilter.Prediction(chrono_2.Split(), &margData2[4]);

        vhclFilter.Correction(&margData1[0], &margData1[7]);
        //headFilter.Correction(&margData2[0], &margData2[7]);
#endif
        interrupts();

        // Get quat rotation difference, store result in tx_buffer[0:3]
        //quatDiv(vhclFilter.GetQuat(), headFilter.GetQuat(), tx_buffer.num_f);
        memcpy(tx_buffer.num_f, vhclFilter.GetQuat(), 4*sizeof(float));
        filterCnt++;
    }
#endif /* AHRS */

    /* Task 2 - USB data TX @ 1 msec */
    if (task_usbTx.check()) {

        tx_buffer.num_d[15] = pktCnt;           // Place pktCnt into last 4 bytes
        num = RawHID.send(tx_buffer.raw, 0);    // Send the packet (to usb controller tx buffer)
        if (num <= 0) task_usbTx.requeue();     // sending failed, re-run the task on next loop

        if (num > 0) {
            pktCnt++;
            lastTx = micros();
        } else if (Debug == 3) {
            Serial.printf("TX: Fail\t%d\n", num);
        }
    }

    /* Task 3 - USB data RX @ on demand  */
    if (RawHID.available()) {
        num = RawHID.recv(rx_buffer.raw, 10);
        if (num > 0) {
            lastRx = micros();

            if (Debug == 3) {
                Serial.printf("TX:\t%d\tRX:\t%d\n", tx_buffer.num_d[15], rx_buffer.num_d[15]);
                Serial.printf("%d\t\t%d\n", lastTx / 1000 % 1000, lastRx / 1000 % 1000);
            }

        } else if (Debug == 3) {
            Serial.printf("\tRX: Fail\t%d\n", num);
        }
    }

#if 1
    /* Task 4 - Debug Output @ 2 sec */
    if (task_dbgOut.check()) {

        if (Debug) {
            // Timings
            Serial.printf("filter rate = %.2f Hz\n", (float)filterCnt / 2.0f, 2);
#ifdef I2C_SPI_TIME
            Serial.printf("MARG: fs = %.2f Hz\n", (float)irsCnt / 2.0f);
            Serial.printf("MARG: I2C/SPI irsFunc speed = %.2f us\n\n", dt / irsCnt);
            dt = 0;
            irsCnt = 0;
#endif /* I2C_SPI_TIME */

            // The current rotation quaternions
            static const float *q1 = vhclFilter.GetQuat();
            static const float *q2 = headFilter.GetQuat();
            static const float *q  = tx_buffer.num_f;
            Serial.printf("q1 = \t%.2f\t%.2f\t%.2f\t%.2f\n",  q1[0], q1[1], q1[2], q1[3]);
            Serial.printf("q2 = \t%.2f\t%.2f\t%.2f\t%.2f\n",  q2[0], q2[1], q2[2], q2[3]);
            Serial.printf("q  = \t%.2f\t%.2f\t%.2f\t%.2f\n\n", q[0],  q[1],  q[2],  q[3]);
        }

        if (Debug == 2) {
            // The current raw sensor data
            Serial.printf("%6.3f\t%6.3f\t%6.3f\n",   margData1[0], margData1[1], margData1[2]);
            Serial.printf("%6.2f\n",                 margData1[3]);
            Serial.printf("%6.3f\t%6.3f\t%6.3f\n",   margData1[4], margData1[5], margData1[6]);
            Serial.printf("%6.2f\t%6.2f\t%6.2f\n\n", margData1[7], margData1[8], margData1[9]);

            Serial.printf("%6.3f\t%6.3f\t%6.3f\n",   margData2[0], margData2[1], margData2[2]);
            Serial.printf("%6.2f\n",                 margData2[3]);
            Serial.printf("%6.3f\t%6.3f\t%6.3f\n",   margData2[4], margData2[5], margData2[6]);
            Serial.printf("%6.2f\t%6.2f\t%6.2f\n\n", margData2[7], margData2[8], margData2[9]);
        }

        // Toggle the LED (signal uC is alive)
        digitalWrite(ledPin, !digitalRead(ledPin));

        // Reset print counters
        filterCnt = 0;
    }
#endif
}
