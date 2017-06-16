/*
 * mpu9250.h
 *
 *  Created on: Jun 12, 2017
 *      Author: may
 */

#ifndef _MPU9250_H_
#define _MPU9250_H_

#include "Arduino.h"
#include "i2c_t3.h"  // I2C library
// Function defines

// The MPU9250 Class
class mpu9250 {

public:
    // Register Map
    // See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
    const uint8_t MS5637_RESET      = 0x1E;
    const uint8_t MS5637_CONVERT_D1 = 0x40;
    const uint8_t MS5637_CONVERT_D2 = 0x50;
    const uint8_t MS5637_ADC_READ   = 0x00;

    // See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
    // above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
    //
    //Magnetometer Registers
    const uint8_t AK8963_ADDRESS   = 0x0C;
    const uint8_t AK8963_WHO_AM_I  = 0x00; // should return 0x48
    const uint8_t AK8963_INFO      = 0x01;
    const uint8_t AK8963_ST1       = 0x02;  // data ready status bit 0
    const uint8_t AK8963_XOUT_L    = 0x03;  // data
    const uint8_t AK8963_XOUT_H    = 0x04;
    const uint8_t AK8963_YOUT_L    = 0x05;
    const uint8_t AK8963_YOUT_H    = 0x06;
    const uint8_t AK8963_ZOUT_L    = 0x07;
    const uint8_t AK8963_ZOUT_H    = 0x08;
    const uint8_t AK8963_ST2       = 0x09;  // Data overflow bit 3 and data read error status bit 2
    const uint8_t AK8963_CNTL      = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
    const uint8_t AK8963_ASTC      = 0x0C;  // Self test control
    const uint8_t AK8963_I2CDIS    = 0x0F;  // I2C disable
    const uint8_t AK8963_ASAX      = 0x10;  // Fuse ROM x-axis sensitivity adjustment value
    const uint8_t AK8963_ASAY      = 0x11;  // Fuse ROM y-axis sensitivity adjustment value
    const uint8_t AK8963_ASAZ      = 0x12;  // Fuse ROM z-axis sensitivity adjustment value

    const uint8_t SELF_TEST_X_GYRO = 0x00;
    const uint8_t SELF_TEST_Y_GYRO = 0x01;
    const uint8_t SELF_TEST_Z_GYRO = 0x02;

    /*
    const uint8_t X_FINE_GAIN      = 0x03; // [7:0] fine gain
    const uint8_t Y_FINE_GAIN      = 0x04;
    const uint8_t Z_FINE_GAIN      = 0x05;
    const uint8_t XA_OFFSET_H      = 0x06; // User-defined trim values for accelerometer
    const uint8_t XA_OFFSET_L_TC   = 0x07;
    const uint8_t YA_OFFSET_H      = 0x08;
    const uint8_t YA_OFFSET_L_TC   = 0x09;
    const uint8_t ZA_OFFSET_H      = 0x0A;
    const uint8_t ZA_OFFSET_L_TC   = 0x0B; */

    const uint8_t SELF_TEST_X_ACCEL = 0x0D;
    const uint8_t SELF_TEST_Y_ACCEL = 0x0E;
    const uint8_t SELF_TEST_Z_ACCEL = 0x0F;

    const uint8_t SELF_TEST_A       = 0x10;

    const uint8_t XG_OFFSET_H      = 0x13; // User-defined trim values for gyroscope
    const uint8_t XG_OFFSET_L      = 0x14;
    const uint8_t YG_OFFSET_H      = 0x15;
    const uint8_t YG_OFFSET_L      = 0x16;
    const uint8_t ZG_OFFSET_H      = 0x17;
    const uint8_t ZG_OFFSET_L      = 0x18;
    const uint8_t SMPLRT_DIV       = 0x19;
    const uint8_t CONFIG           = 0x1A;
    const uint8_t GYRO_CONFIG      = 0x1B;
    const uint8_t ACCEL_CONFIG     = 0x1C;
    const uint8_t ACCEL_CONFIG2    = 0x1D;
    const uint8_t LP_ACCEL_ODR     = 0x1E;
    const uint8_t WOM_THR          = 0x1F;

    const uint8_t MOT_DUR          = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
    const uint8_t ZMOT_THR         = 0x21;  // Zero-motion detection threshold bits [7:0]
    const uint8_t ZRMOT_DUR        = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

    const uint8_t FIFO_EN          = 0x23;
    const uint8_t I2C_MST_CTRL     = 0x24;
    const uint8_t I2C_SLV0_ADDR    = 0x25;
    const uint8_t I2C_SLV0_REG     = 0x26;
    const uint8_t I2C_SLV0_CTRL    = 0x27;
    const uint8_t I2C_SLV1_ADDR    = 0x28;
    const uint8_t I2C_SLV1_REG     = 0x29;
    const uint8_t I2C_SLV1_CTRL    = 0x2A;
    const uint8_t I2C_SLV2_ADDR    = 0x2B;
    const uint8_t I2C_SLV2_REG     = 0x2C;
    const uint8_t I2C_SLV2_CTRL    = 0x2D;
    const uint8_t I2C_SLV3_ADDR    = 0x2E;
    const uint8_t I2C_SLV3_REG     = 0x2F;
    const uint8_t I2C_SLV3_CTRL    = 0x30;
    const uint8_t I2C_SLV4_ADDR    = 0x31;
    const uint8_t I2C_SLV4_REG     = 0x32;
    const uint8_t I2C_SLV4_DO      = 0x33;
    const uint8_t I2C_SLV4_CTRL    = 0x34;
    const uint8_t I2C_SLV4_DI      = 0x35;
    const uint8_t I2C_MST_STATUS   = 0x36;
    const uint8_t INT_PIN_CFG      = 0x37;
    const uint8_t INT_ENABLE       = 0x38;
    const uint8_t DMP_INT_STATUS   = 0x39;  // Check DMP interrupt
    const uint8_t INT_STATUS       = 0x3A;
    const uint8_t ACCEL_XOUT_H     = 0x3B;
    const uint8_t ACCEL_XOUT_L     = 0x3C;
    const uint8_t ACCEL_YOUT_H     = 0x3D;
    const uint8_t ACCEL_YOUT_L     = 0x3E;
    const uint8_t ACCEL_ZOUT_H     = 0x3F;
    const uint8_t ACCEL_ZOUT_L     = 0x40;
    const uint8_t TEMP_OUT_H       = 0x41;
    const uint8_t TEMP_OUT_L       = 0x42;
    const uint8_t GYRO_XOUT_H      = 0x43;
    const uint8_t GYRO_XOUT_L      = 0x44;
    const uint8_t GYRO_YOUT_H      = 0x45;
    const uint8_t GYRO_YOUT_L      = 0x46;
    const uint8_t GYRO_ZOUT_H      = 0x47;
    const uint8_t GYRO_ZOUT_L      = 0x48;

    const uint8_t EXT_SENS_DATA_00   = 0x49;
    const uint8_t EXT_SENS_DATA_01   = 0x4A;
    const uint8_t EXT_SENS_DATA_02   = 0x4B;
    const uint8_t EXT_SENS_DATA_03   = 0x4C;
    const uint8_t EXT_SENS_DATA_04   = 0x4D;
    const uint8_t EXT_SENS_DATA_05   = 0x4E;
    const uint8_t EXT_SENS_DATA_06   = 0x4F;
    const uint8_t EXT_SENS_DATA_07   = 0x50;
    const uint8_t EXT_SENS_DATA_08   = 0x51;
    const uint8_t EXT_SENS_DATA_09   = 0x52;
    const uint8_t EXT_SENS_DATA_10   = 0x53;
    const uint8_t EXT_SENS_DATA_11   = 0x54;
    const uint8_t EXT_SENS_DATA_12   = 0x55;
    const uint8_t EXT_SENS_DATA_13   = 0x56;
    const uint8_t EXT_SENS_DATA_14   = 0x57;
    const uint8_t EXT_SENS_DATA_15   = 0x58;
    const uint8_t EXT_SENS_DATA_16   = 0x59;
    const uint8_t EXT_SENS_DATA_17   = 0x5A;
    const uint8_t EXT_SENS_DATA_18   = 0x5B;
    const uint8_t EXT_SENS_DATA_19   = 0x5C;
    const uint8_t EXT_SENS_DATA_20   = 0x5D;
    const uint8_t EXT_SENS_DATA_21   = 0x5E;
    const uint8_t EXT_SENS_DATA_22   = 0x5F;
    const uint8_t EXT_SENS_DATA_23   = 0x60;
    const uint8_t MOT_DETECT_STATUS  = 0x61;
    const uint8_t I2C_SLV0_DO        = 0x63;
    const uint8_t I2C_SLV1_DO        = 0x64;
    const uint8_t I2C_SLV2_DO        = 0x65;
    const uint8_t I2C_SLV3_DO        = 0x66;
    const uint8_t I2C_MST_DELAY_CTRL = 0x67;
    const uint8_t SIGNAL_PATH_RESET  = 0x68;
    const uint8_t MOT_DETECT_CTRL    = 0x69;
    const uint8_t USER_CTRL          = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP
    const uint8_t PWR_MGMT_1         = 0x6B;  // Device defaults to the SLEEP mode
    const uint8_t PWR_MGMT_2         = 0x6C;

    const uint8_t DMP_BANK         = 0x6D;    // Activates a specific bank in the DMP
    const uint8_t DMP_RW_PNT       = 0x6E;    // Set read/write pointer to a specific start address in specified DMP bank
    const uint8_t DMP_REG          = 0x6F;    // Register in DMP from which to read or to which to write
    const uint8_t DMP_REG_1        = 0x70;
    const uint8_t DMP_REG_2        = 0x71;
    const uint8_t FIFO_COUNTH      = 0x72;
    const uint8_t FIFO_COUNTL      = 0x73;
    const uint8_t FIFO_R_W         = 0x74;
    const uint8_t WHO_AM_I_MPU9250 = 0x75; // Should return = 0x71
    const uint8_t XA_OFFSET_H      = 0x77;
    const uint8_t XA_OFFSET_L      = 0x78;
    const uint8_t YA_OFFSET_H      = 0x7A;
    const uint8_t YA_OFFSET_L      = 0x7B;
    const uint8_t ZA_OFFSET_H      = 0x7D;
    const uint8_t ZA_OFFSET_L      = 0x7E;

    const uint8_t MPU9250_ADDRESS = 0x68;  // Device address when ADO = 0

    // Configuration parameters
    enum Ascale {
        AFS_2G = 0,
        AFS_4G,
        AFS_8G,
        AFS_16G
    };

    enum Gscale {
        GFS_250DPS = 0,
        GFS_500DPS,
        GFS_1000DPS,
        GFS_2000DPS
    };

    enum Mscale {
        MFS_14BITS = 0, // 0.6 mG per LSB
        MFS_16BITS      // 0.15 mG per LSB
    };

    enum Mmode {
        MRATE_8HZ = 0x02,
        MRATE_100HZ = 0x06
    };

    // Member Variables
    float _magCalFactory[3] = {0, 0, 0}, _magScale[3]  = {0, 0, 0};  // Factory mag calibration and mag scale
    float _gyroBias[3] = {0, 0, 0}, _accelBias[3] = {0, 0, 0}, _magBias[3] = {0, 0, 0}; // Bias corrections

    // Specify sensor full scale
    uint8_t _Gscale, _Ascale, _Mscale, _Mmode;  // choosen full scale
    float _aRes, _gRes, _mRes;                  // scale resolutions factor per LSB for the sensors

    bool _newMagData = false;

    //int16_t _MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
    //int16_t _accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
    //int16_t _gyroCount[3];   // Stores the 16-bit signed gyro sensor output
    //int16_t _magCount[3];    // Stores the 16-bit signed magnetometer sensor output

    //uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
    //float pitch, yaw, roll;
    //float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
    //float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
    //uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
    //uint32_t Now = 0;                         // used to calculate integration interval
    //float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
    //float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
    //float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
    //float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

    // Class functions
    void InitAK8963(float * destination);
    void Init();

    void SetAres(uint8_t scale);
    void SetGres(uint8_t scale);
    void SetMres(uint8_t scale);
    void SetMmode(uint8_t mode);

    void SelfTest(float * destination);
    void MagCal(float * dest1, float * dest2);
    void AcelGyroCal(float * dest1, float * dest2);
    void SetMagCal(float *magBias, float *magScale);

    int16_t ReadTempData();
    void ReadMagData(int16_t * destination);
    void ReadAccelData(int16_t * destination);
    void ReadGyroData(int16_t * destination);
    void ReadMPU9250Data(int16_t * destination);

    void WriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t ReadByte(uint8_t address, uint8_t subAddress);
    void ReadBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

    bool NewMagData();
};
#endif /* _MPU9250_H_ */
