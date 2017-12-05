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

// Debug flag
// 0: off, 1: std, 2: verbose, 3: vverbose
static const int Debug = 1;

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

FusionFilter filter;
volatile int int1_event = 0, int2_event = 0;
volatile uint32_t start_millis;

#if defined(EVAL_FILTER)
FusionFilter filter_a, filter_b;
Stopwatch chrono_a;
#endif /* EVAL_FILTER */

Stopwatch chrono_1;

MetroExt task_usbTx  = MetroExt(1000);      //   1 msec
MetroExt task_dbgOut = MetroExt(2000000);   //   2 sec

// Ping-pong data buffers
static struct marg_t {
    float accel[3];
    float temp;
    float gyro[3];
    float mag[3];
} margData1[2], margData2[2];
volatile int PP1 = 0, PP2 = 0; // ping-pong

void irs1Func_vhcl()
{
    vhclMarg.GetAll(margData1[!PP1].accel, HS_TRUE);

    int1_event = 1;
}

void irs2Func_head()
{
#ifdef I2C_SPI_TIME
    ts = micros();
#endif /* I2C_SPI_TIME */

    headMarg.GetAll(margData2[!PP2].accel, HS_TRUE);

#ifdef I2C_SPI_TIME
    dt += micros() - ts;
    irsCnt++;
#endif /* I2C_SPI_TIME */

    int2_event = 1;
}

void setup()
{
#if 1
    // Pre-calibrated values (office)
    float magHardIron[][3] = { {53.000000f, 59.000000f, -76.000000f},
                               {13.000000f, 262.000000f, -187.000000f} };
    float magSoftIron[][3] = { {1.030303f, 1.002950f, 0.968661f},
                               {1.080303f, 0.907125f, 1.028860f} };
#else
    // Pre-calibrated values (office-old)
    float magHardIron[][3] = { {36.000000f, 59.000000f, -52.000000f},
                               {-1.000000f, 258.000000f, -179.000000f} };
    float magSoftIron[][3] = { {1.112245f, 0.981982f, 0.923729f},
                               {1.137931f, 0.895349f, 0.995690f} };
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

    if (Debug) Serial.printf("MPU9250 (1): Calibrating and Initialising ...\n");

    // Config for normal operation, set sample-rate div to '0x0X + 1'
    vhclMarg.Init(ACCEL_RANGE_2G, GYRO_RANGE_250DPS, 0x00);

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

    if (Debug) Serial.printf("MPU9250 (2): 9-axis motion sensor is online\n");

    // Start by performing self test (and reporting values)
    headMarg.SelfTest();
    delay(1000);

    if (Debug) Serial.printf("MPU9250 (2): Calibrating and Initialising ...\n");

    // Config for normal operation, set sample-rate div to '0x0X + 1'
    headMarg.Init(ACCEL_RANGE_2G, GYRO_RANGE_250DPS, 0x00);

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
    vhclMarg.EnableInterrupt(intPin1_vhcl, irs1Func_vhcl);
    headMarg.EnableInterrupt(intPin2_head, irs2Func_head);
    interrupts();

    // Wait for the host application to be ready
    while (!RawHID.available());

#if defined(EVAL_FILTER)
    filter_a._alpha = filter_a._beta = 0.0f;
    filter_b._alpha = filter_b._beta = 1.0f;
    chrono_a.Reset();
    Serial.printf("start_millis is = %d\n", start_millis);
#endif /* EVAL_FILTER */

    start_millis = millis();

    chrono_1.Reset();
}

void loop()
{
    static uint32_t filterCnt = 0;                          // loop counter
    static data_t rx_buffer = { 0.0f };                     // usb rx buffer zero initialized
    static data_t tx_buffer = { 1.0f, 0.0f, 0.0f, 0.0f };   // usb tx buffer unit_quat initialized
    static uint8_t num = 0;                                 // usb return code
    static uint32_t lastTx = 0, lastRx = 0;                 // last rx/tx time of usb data
    static uint32_t pktCnt = 0;                             // usb packet number
    static float Ts_max = 0;                                // fusion speed variable

    /* Task 1 - Filter sensor data @ Interrupt rate (1 kHz) */
    if (int1_event && int2_event) {

        // Start normal filter operation after 5s
        if (millis() - start_millis > 5000) {
            filter._alpha = 0.0005f;
            filter._beta = 0.005f;
        }

        noInterrupts();
        PP1 = !PP1; // ping-pong
        PP2 = !PP2;

        Stopwatch chrono_3;

#if defined(EVAL_FILTER)
        static float margData_ab2[10];
        memcpy(margData_ab2, margData2, sizeof(margData_ab2));

        // Stop correcting filter_a 30 secs after start
        if (millis() - start_millis < 30000)
            filter_a.SetQuat(filter.GetQuat());

        filter_a.Prediction(&margData1[4], &margData_ab2[4], chrono_a.Split());

        filter_b.Correction(&margData1[0], &margData1[7], &margData_ab2[0], &margData_ab2[7],
                          vhclMarg._magReady, headMarg._magReady);
#else
        // Copy raw sensor data to tx_buffer
        memcpy(&tx_buffer.num_f[4], margData2[PP2].accel, 10*sizeof(float));
#endif /* EVAL_FILTER */

        filter.Prediction(margData1[PP1].gyro,  margData2[PP2].gyro, chrono_1.Split());
        filter.Correction(margData1[PP1].accel, margData1[PP1].mag,
                          margData2[PP2].accel, margData2[PP2].mag,
                          vhclMarg._magReady, headMarg._magReady);

        int1_event = int2_event = 0;
        Ts_max += chrono_3.Split();
        interrupts();

        memcpy(tx_buffer.num_f, filter.GetQuat(), 4*sizeof(float));

#if defined(EVAL_FILTER)
        memcpy(&tx_buffer.num_f[4], filter_a.GetQuat(), 4*sizeof(float));
        memcpy(&tx_buffer.num_f[8], filter_b.GetQuat(), 4*sizeof(float));
#endif /* EVAL_FILTER*/

        filterCnt++;
    }

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
            Serial.printf("   max rate = %.2f Hz\n", (float)filterCnt / Ts_max);
            Ts_max = 0.0f;

#ifdef I2C_SPI_TIME
            Serial.printf("MARG: fs = %.2f Hz\n", (float)irsCnt / 2.0f);
            Serial.printf("MARG: I2C/SPI irsFunc speed = %.2f us\n\n", dt / irsCnt);
            dt = 0;
            irsCnt = 0;
#endif /* I2C_SPI_TIME */

            // The current rotation quaternions
            static const float *q  = tx_buffer.num_f;
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
