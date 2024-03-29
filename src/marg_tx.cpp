/*
 * imusensTX.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: may
 */

#include <Arduino.h>
#include <Metro.h>
#include <SPI.h>

#include "mpu9250.h"
#include "MetroExt.h"
#include "Stopwatch.h"
#include "FusionFilter.h"

// Debug flag
// 0: off, 1: std, 2: verbose, 3: vverbose
static const int Debug = 1;

#define GOLF // Using mag. calibration for vw golf

// Pin definitions
static const int ledPin = 13;
static const int intPin1 = 9;  // MPU9250 1 vhcl intPin
static const int intPin2= 18;  // MPU9250 2 head intPin

#ifdef I2C_SPI_TIME
// Times the avg sensor readout time
volatile static int irsCnt = 0;
volatile static float dt = 0, ts = 0;
#endif

// Globals
mpu9250 vhclMarg(10, MOSI_PIN_28);       // MPU9250 1 On SPI bus 0 at csPin 10
mpu9250 headMarg(17, MOSI_PIN_21);       // MPU9250 2 On SPI bus 1 at csPin 17

FusionFilter filter;
stopwatch chrono_1;
volatile int int1_event = 0, int2_event = 0;

Metro    task_dbgOut(2000); // 2000 millis
MetroExt task_usbTx(1000);  // 1000 micros

// marg data buffer
typedef struct marg {
    float accel[3];
    float temp;
    float gyro[3];
    float mag[3];
    int   magRdy;
} marg_t;

// 64 byte usb packet
typedef union data {
  float num_f[16];
  uint32_t num_d[16];
  byte raw[64];
} data_t;

void irs1_func()
{
    int1_event = 1;
}

void irs2_func()
{
    int2_event = 1;
}

void setup()
{
#if defined(GOLF)
    // Pre-calibrated values (golf)
    float magHardIron[][3] = { {51.000000f, 63.000000f, -83.000000f},
                               {10.000000f, 276.000000f, -188.000000f} };
    float magSoftIron[][3] = { {0.975575f, 1.047840f, 0.979798f},
                               {1.061475f, 0.938406f, 1.007782f} };
#else
    // Pre-calibrated values (home)
    float magHardIron[][3] = { {58.000000f, 59.000000f, -79.000000f},
                               {18.000000f, 282.000000f, -161.000000f} };
    float magSoftIron[][3] = { {0.998779f, 1.032828f, 0.970344f},
                               {1.007547f, 0.946809f, 1.051181f} };
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

    if (Debug) Serial.printf("MPU9250 (1): 9-axis MARG sensor is online\n");

    // Config for normal operation, set sample-rate div to '0x0X + 1'
    vhclMarg.Init(ACCEL_RANGE_2G, GYRO_RANGE_500DPS, 0x00);

    // Check if AK8963 magnetometer is online
    if (Debug) Serial.printf("AK8963  (1): I'm 0x%02x\n", vhclMarg.whoAmIAK8963());

    // Get magnetometer calibration from AK8963 ROM
    vhclMarg.InitAK8963(MAG_RANGE_16BIT, MAG_RATE_100HZ);

#ifdef RESET_MAGCAL
    // TODO: This needs to be automated/triggered on demand by the user
    Serial.printf("AK8963  (1) initialized for active data mode ...\n");
    Serial.printf("Mag Calibration: Wave device in a figure eight until done!\n");
    delay(4000);
    vhclMarg.MagCal();
#else
    if (Debug) Serial.printf("AK8963  (1): Mag calibration using pre-recorded values\n");
    vhclMarg.SetMagCal(&magHardIron[0][0], &magSoftIron[0][0]);
#endif /* RESET_MAGCAL */

next:
    /* IMU 2 (Head) Setup */
    // Setup bus specifics for this device
    headMarg.WireSetup();

    // Start the bus (only one device needs to start it on a shared bus)
    headMarg.WireBegin();

    // Skip setup if this MPU9250 is not online
    if (headMarg.whoAmI() != 0x71) goto end;

    if (Debug) Serial.printf("MPU9250 (2): 9-axis MARG sensor is online\n");

    // Config for normal operation, set sample-rate div to '0x0X + 1'
    headMarg.Init(ACCEL_RANGE_2G, GYRO_RANGE_500DPS, 0x00);

    // Check if AK8963 magnetometer is online
    if (Debug) Serial.printf("AK8963  (2): I'm 0x%02x\n", headMarg.whoAmIAK8963());

    // Get magnetometer calibration from AK8963 ROM
    headMarg.InitAK8963(MAG_RANGE_16BIT, MAG_RATE_100HZ);

#ifdef RESET_MAGCAL
    // TODO: This needs to be automated/triggered on demand by the user
    Serial.printf("AK8963  (2) initialized for active data mode...\n");
    Serial.printf("Mag Calibration: Wave device in a figure eight until done!\n");
    delay(4000);
    headMarg.MagCal();
#else
    if (Debug) Serial.printf("AK8963  (2): Mag calibration using pre-recorded values\n");
    headMarg.SetMagCal(&magHardIron[1][0], &magSoftIron[1][0]);
#endif /* RESET_MAGCAL */

end:
    if (Debug) Serial.printf("\nSetup done, enabling interrupts!\n");

    // Enable interrupts
    vhclMarg.EnableInterrupt(intPin1, irs1_func);
    headMarg.EnableInterrupt(intPin2, irs2_func);
    interrupts();

    // Wait for the host application to be ready
    while (!RawHID.available());

    chrono_1.reset();
}

void loop()
{
    static int do_once = 1;
    static data_t rx_buffer = { 0 };                        // usb rx buffer
    static data_t tx_buffer = { 1.0f, 0.0f, 0.0f, 0.0f };   // usb tx buffer unit_quat initialized
    static marg_t marg1, marg2;                             // marg sensor data structs

    // These are mostly for debugging
    static uint32_t filterCnt = 0;                          // loop counter
    static float Ts_max = 0;                                // fusion speed variable

    // Start normal filter operation after 2s
    if (do_once && chrono_1.peek_lp() > 2.0f) {
        filter.SetGains(0.001f, 0.001f);
        do_once = 0;
    }

    /* Task 1 - Get MARG1 data */
    if (int1_event) {
        vhclMarg.GetAll(marg1.accel, HS_TRUE);
        int1_event = 0;
    }

    /* Task 2 - Get MARG2 data */
    if (int2_event) {
#ifdef I2C_SPI_TIME
        ts = micros();
#endif /* I2C_SPI_TIME */
        headMarg.GetAll(marg2.accel, HS_TRUE);
#ifdef I2C_SPI_TIME
        dt += micros() - ts;
        irsCnt++;
#endif /* I2C_SPI_TIME */
        int2_event = 0;

        // Copy raw sensor data to tx_buffer
        memcpy(&tx_buffer.num_f[4], marg2.accel, 10*sizeof(float));

        stopwatch chrono_2;

        /* Task 3 - Sensorfusion */
        filter.Prediction(marg1.gyro, marg2.gyro, chrono_1.split());
        filter.Correction(marg1.accel, marg1.mag, marg2.accel, marg2.mag, marg1.magRdy, marg2.magRdy);

        Ts_max += chrono_2.split();     // time spent filtering

        // Copy quat to tx_buffer
        memcpy(tx_buffer.num_f, filter.GetQuat(), 4*sizeof(float));
        filterCnt++;
    }

    /* Task 4 - USB data TX @ 1 msec */
    if (task_usbTx.check()) {

        tx_buffer.num_d[15] = millis();             // Packet TX time into last 4 bytes
        int num = RawHID.send(tx_buffer.raw, 0);    // Send the packet (to usb controller tx buffer)
        if (num <= 0) task_usbTx.requeue();         // sending failed, re-run the task on next loop
    }

    /* Task 5 - USB data RX @ on demand  */
    if (RawHID.available()) {
        int num = RawHID.recv(rx_buffer.raw, 10);
        if (Debug && num > 0) {
            Serial.printf("Received bitfield: 0x%02x\n\n",rx_buffer.raw[0]);
        }
    }

#if 1
    /* Task 6 - Debug Output @ 2 sec */
    if (task_dbgOut.check()) {

        if (Debug) {
            // Timings
            Serial.printf("filter rate = %.2f Hz\n", (float)filterCnt / 2.0f, 2);
            Serial.printf("   max rate = %.2f Hz\n", (float)filterCnt / Ts_max);
            Ts_max = 0.0f;

#ifdef I2C_SPI_TIME
            Serial.printf("MARG: fs = %.2f Hz\n", (float)irsCnt / 2.0f);
            Serial.printf("MARG: I2C/SPI irsFunc speed = %.2f us\n\n", dt / irsCnt);
            dt = 0;
            irsCnt = 0;
#endif /* I2C_SPI_TIME */

            // The current rotation quaternion
            const float *q  = tx_buffer.num_f;
            Serial.printf("q  = \t%.2f\t%.2f\t%.2f\t%.2f\n\n", q[0],  q[1],  q[2],  q[3]);
        }

        if (Debug == 2) {
            marg_t *m = &marg1;
            // The current raw sensor data
            Serial.printf("%6.3f\t%6.3f\t%6.3f\n",   m->accel[0], m->accel[1], m->accel[2]);
            Serial.printf("%6.2f\n",                 m->temp);
            Serial.printf("%6.3f\t%6.3f\t%6.3f\n",   m->gyro[0],  m->gyro[1],  m->gyro[2]);
            Serial.printf("%6.2f\t%6.2f\t%6.2f\n\n", m->mag[0],   m->mag[1],   m->mag[2]);

            m = &marg2;
            Serial.printf("%6.3f\t%6.3f\t%6.3f\n",   m->accel[0], m->accel[1], m->accel[2]);
            Serial.printf("%6.2f\n",                 m->temp);
            Serial.printf("%6.3f\t%6.3f\t%6.3f\n",   m->gyro[0],  m->gyro[1],  m->gyro[2]);
            Serial.printf("%6.2f\t%6.2f\t%6.2f\n\n", m->mag[0],   m->mag[1],   m->mag[2]);
        }

        // Toggle the LED (signal uC is alive)
        digitalWrite(ledPin, !digitalRead(ledPin));

        // Reset print counters
        filterCnt = 0;
    }
#endif
}
