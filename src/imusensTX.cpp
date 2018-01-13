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
#include "mpu9250.h"
#include "MetroExt.h"
#include "Stopwatch.h"
#include "FusionFilter.h"

#define I2C_SPI_TIME
//#define RESET_MAGCAL

// Debug flag
// 0: off, 1: std, 2: verbose, 3: vverbose
static const int Debug = 1;

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
Stopwatch chrono_1;
uint32_t start_millis;
volatile int int1_event = 0, int2_event = 0;

#if defined(EVAL_FILTER)
FusionFilter filter_a, filter_b;
Stopwatch chrono_a;
#endif /* EVAL_FILTER */

MetroExt task_filter = MetroExt(1000);      // 1 msec
MetroExt task_usbTx  = MetroExt(1000);      // 1 msec
MetroExt task_dbgOut = MetroExt(2000000);   // 2 sec

// Data buffers types
typedef struct marg {
    float accel[3];
    float temp;
    float gyro[3];
    float mag[3];
    int   magRdy;
} marg_t;

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
#if 1
    // Pre-calibrated values (golf)
    float magHardIron[][3] = { {51.000000f, 84.000000f, -97.000000f},
                               {49.000000f, 214.000000f, -27.000000f} };
    float magSoftIron[][3] = { {1.050794, 1.060897, 0.970344f},
                               {1.088972f, 1.027187f, 0.902388f} };
#else
    // Pre-calibrated values (home)
    float magHardIron[][3] = { {58.000000f, 59.000000f, -79.000000f},
                               {18.000000f, 282.000000f, -161.000000f} };
    float magSoftIron[][3] = { {0.998779f, 1.032828f, 0.970344f},
                               {1.007547f, 0.946809f, 1.051181f} };
/*
    // Pre-calibrated values (office)
    float magHardIron[][3] = { {62.000000f, 55.000000f, -80.000000f},
                               {27.000000f, 270.000000f, -188.000000f} };
    float magSoftIron[][3] = { {1.021021f, 1.011905f, 0.968661f},
                               {1.131173f, 0.904938f, 0.989204f} };
*/
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

    // Start by performing self test (and reporting values)
    vhclMarg.SelfTest();
    delay(1000);

    if (Debug) Serial.printf("MPU9250 (1): Calibrating and Initialising ...\n");

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

    // Start by performing self test (and reporting values)
    headMarg.SelfTest();
    delay(1000);

    if (Debug) Serial.printf("MPU9250 (2): Calibrating and Initialising ...\n");

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

#if defined(EVAL_FILTER)
    filter_a._alpha = filter_a._beta = 0.0f;
    filter_b._alpha = filter_b._beta = 1.0f;
    chrono_a.Reset();
#endif /* EVAL_FILTER */

    start_millis = millis();
    chrono_1.Reset();
}

void loop()
{
    static data_t rx_buffer;                                // usb rx buffer
    static data_t tx_buffer = { 1.0f, 0.0f, 0.0f, 0.0f };   // usb tx buffer unit_quat initialized
    static marg_t marg1, marg2;                             // marg sensor data structs

    // These are mostly for debugging
    static uint32_t filterCnt = 0;                          // loop counter
    static uint32_t lastTx = 0, lastRx = 0;                 // last rx/tx time of usb data
    static uint32_t pktCnt = 0;                             // usb packet number
    static float Ts_max = 0;                                // fusion speed variable

    // Start normal filter operation after 2s
    if (millis() - start_millis > 2000) {
        filter._alpha = 0.001f;
        filter._beta = 0.001f;
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
//    }
//
//    /* Task 3 - Filter sensor data @ Interrupt rate (1 kHz) */
//    if (task_filter.check()) {

        Stopwatch chrono_2;
#if defined(EVAL_FILTER)
        static marg_t marg_ab2;
        memcpy(marg_ab2.accel, marg2.accel, sizeof(marg_ab2));

        // Stop correcting filter_a 30 secs after start
        if (millis() - start_millis < 30000)
            filter_a.SetQuat(filter.GetQuat());

        filter_a.Prediction(marg1.gyro, marg_ab2.gyro, chrono_a.Split());

        filter_b.Correction(marg1.accel, marg1.mag, marg_ab2.accel, marg_ab2.mag,
                          marg1.magRdy, marg2.magRdy);
#else
        // Copy raw sensor data to tx_buffer
        memcpy(&tx_buffer.num_f[4], marg2.accel, 10*sizeof(float));
#endif /* EVAL_FILTER */

        filter.Prediction(marg1.gyro, marg2.gyro, chrono_1.Split());
        filter.Correction(marg1.accel, marg1.mag, marg2.accel, marg2.mag, marg1.magRdy, marg2.magRdy);

        Ts_max += chrono_2.Split();

        memcpy(tx_buffer.num_f, filter.GetQuat(), 4*sizeof(float));

#if defined(EVAL_FILTER)
        memcpy(&tx_buffer.num_f[4], filter_a.GetQuat(), 4*sizeof(float));
        memcpy(&tx_buffer.num_f[8], filter_b.GetQuat(), 4*sizeof(float));
#endif /* EVAL_FILTER*/

        filterCnt++;
    }

    /* Task 4 - USB data TX @ 1 msec */
    if (task_usbTx.check()) {

        tx_buffer.num_d[15] = pktCnt;               // Place pktCnt into last 4 bytes
        int num = RawHID.send(tx_buffer.raw, 0);    // Send the packet (to usb controller tx buffer)
        if (num <= 0) task_usbTx.requeue();         // sending failed, re-run the task on next loop

        if (num > 0) {
            pktCnt++;
            lastTx = micros();
        } else if (Debug == 3) {
            Serial.printf("TX: Fail\t%d\n", num);
        }
    }

    /* Task 5 - USB data RX @ on demand  */
    if (RawHID.available()) {
        int num = RawHID.recv(rx_buffer.raw, 10);
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

            // The current rotation quaternions
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
