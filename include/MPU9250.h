/*
MPU9250.h
Brian R Taylor
brian.taylor@bolderflight.com
2017-01-03

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "i2c_t3.h"  // I2C library

#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN
    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
    enum spi_mosi_pin
    {
      MOSI_PIN_7,
      MOSI_PIN_11
    };
    #endif
    // Teensy 3.5 || Teensy 3.6
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    enum spi_mosi_pin
    {
      MOSI_PIN_0,
      MOSI_PIN_7,
      MOSI_PIN_11,
      MOSI_PIN_21,
      MOSI_PIN_28,
      MOSI_PIN_44,
      MOSI_PIN_52
    };
    #endif
    // Teensy LC 
    #if defined(__MKL26Z64__)
    enum spi_mosi_pin
    {
      MOSI_PIN_0,
      MOSI_PIN_7,
      MOSI_PIN_11,
      MOSI_PIN_21
    };
    #endif
#endif

enum mpu9250_gyro_range
{
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
};

enum mpu9250_accel_range
{
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
};

enum mpu9250_dlpf_bandwidth
{
    DLPF_BANDWIDTH_184HZ,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ
};

class MPU9250{
    public:
        MPU9250(uint8_t address, uint8_t bus);
        MPU9250(uint8_t address, uint8_t bus, i2c_pins pins);
        MPU9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups);
        MPU9250(uint8_t csPin);
        MPU9250(uint8_t csPin, spi_mosi_pin pin);
        int begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange);
        int setFilt(mpu9250_dlpf_bandwidth bandwidth, uint8_t SRD);
        int enableInt(bool enable);
        void getAccel(float* ax, float* ay, float* az);
        void getGyro(float* gx, float* gy, float* gz);
        void getMag(float* hx, float* hy, float* hz);
        void getTemp(float *t);
        void getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
        void getMotion7(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* t);
        void getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz);
        void getMotion10(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz, float* t);

        void getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az);
        void getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz);
        void getMagCounts(int16_t* hx, int16_t* hy, int16_t* hz);
        void getTempCounts(int16_t* t);
        void getMotion6Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getMotion7Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t);
        void getMotion9Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz);
        void getMotion10Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz, int16_t* t);

        // may: extra functions
        void accelGyroCal();
        void magCal(); 
        void selfTest(); 
    
    private:
        uint8_t _address;
        uint8_t _bus;
        i2c_pins _pins;
        i2c_pullup _pullups;
        bool _userDefI2C;
        uint8_t _csPin;
        spi_mosi_pin _mosiPin;
        bool _useSPI;
        bool _useSPIHS;
        float _accelScale;
        float _gyroScale;
        float _magScaleX, _magScaleY, _magScaleZ;
        const float _tempScale = 333.87f;
        const float _tempOffset = 21.0f;

        // SPI constants
        const uint8_t SPI_READ = 0x80;
        const uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
        const uint32_t SPI_HS_CLOCK = 20000000; // 20 MHz

        // i2c bus frequency
        const uint32_t _i2cRate = 400000;

        // constants
        const float G = 9.807f;
        const float _d2r = 3.14159265359f/180.0f;

        // may: extra variables
        // Bias corrections for gyro and accelerometer
        int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
        int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
        int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
        
        float _magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
        float _gyroBias[3] = {0, 0, 0}; 
        float _accelBias[3] = {0, 0, 0}; 
        float _magBias[3] = {0, 0, 0}, _magScale[3]  = {0, 0, 0};

        float _selfTest[6];            // holds results of gyro and accelerometer self test

        // MPU9250 registers
        const uint8_t SELF_TEST_X_GYRO = 0x00;
        const uint8_t SELF_TEST_Y_GYRO = 0x01;
        const uint8_t SELF_TEST_Z_GYRO = 0x02;
        const uint8_t SELF_TEST_X_ACCEL = 0x0D;
        const uint8_t SELF_TEST_Y_ACCEL = 0x0E;
        const uint8_t SELF_TEST_Z_ACCEL = 0x0F;

        const uint8_t XG_OFFSET_H = 0x13;  // User-defined trim values for gyroscope
        const uint8_t XG_OFFSET_L = 0x14;
        const uint8_t YG_OFFSET_H = 0x15;
        const uint8_t YG_OFFSET_L = 0x16;
        const uint8_t ZG_OFFSET_H = 0x17;
        const uint8_t ZG_OFFSET_L = 0x18;

        const uint8_t SMPLRT_DIV = 0x19;
        const uint8_t CONFIG = 0x1A;
        const uint8_t GYRO_CONFIG = 0x1B;
        const uint8_t ACCEL_CONFIG = 0x1C;
        const uint8_t ACCEL_CONFIG2 = 0x1D;
        const uint8_t LP_ACCEL_ODR = 0x1E;
        const uint8_t WOM_THR = 0x1F;

        const uint8_t MOT_DUR        = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
        const uint8_t ZMOT_THR       = 0x21;  // Zero-motion detection threshold bits [7:0]
        const uint8_t ZRMOT_DUR      = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

        const uint8_t FIFO_EN         = 0x23;
        const uint8_t I2C_MST_CTRL    = 0x24;
        const uint8_t I2C_SLV0_ADDR   = 0x25;
        const uint8_t I2C_SLV0_REG    = 0x26;
        const uint8_t I2C_SLV0_CTRL   = 0x27;
        const uint8_t I2C_SLV1_ADDR   = 0x28;
        const uint8_t I2C_SLV1_REG    = 0x29;
        const uint8_t I2C_SLV1_CTRL   = 0x2A;
        const uint8_t I2C_SLV2_ADDR   = 0x2B;
        const uint8_t I2C_SLV2_REG    = 0x2C;
        const uint8_t I2C_SLV2_CTRL   = 0x2D;
        const uint8_t I2C_SLV3_ADDR   = 0x2E;
        const uint8_t I2C_SLV3_REG    = 0x2F;
        const uint8_t I2C_SLV3_CTRL   = 0x30;
        const uint8_t I2C_SLV4_ADDR   = 0x31;
        const uint8_t I2C_SLV4_REG    = 0x32;
        const uint8_t I2C_SLV4_DO     = 0x33;
        const uint8_t I2C_SLV4_CTRL   = 0x34;
        const uint8_t I2C_SLV4_DI     = 0x35;
        const uint8_t I2C_MST_STATUS  = 0x36;
        const uint8_t INT_PIN_CFG     = 0x37;
        const uint8_t INT_ENABLE      = 0x38;
        const uint8_t DMP_INT_STATUS  = 0x39;  // Check DMP interrupt
        const uint8_t INT_STATUS      = 0x3A;
        const uint8_t ACCEL_XOUT_H    = 0x3B;
        const uint8_t ACCEL_XOUT_L    = 0x3C;
        const uint8_t ACCEL_YOUT_H    = 0x3D;
        const uint8_t ACCEL_YOUT_L    = 0x3E;
        const uint8_t ACCEL_ZOUT_H    = 0x3F;
        const uint8_t ACCEL_ZOUT_L    = 0x40;
        const uint8_t TEMP_OUT_H      = 0x41;
        const uint8_t TEMP_OUT_L      = 0x42;
        const uint8_t GYRO_XOUT_H     = 0x43;
        const uint8_t GYRO_XOUT_L     = 0x44;
        const uint8_t GYRO_YOUT_H     = 0x45;
        const uint8_t GYRO_YOUT_L     = 0x46;
        const uint8_t GYRO_ZOUT_H     = 0x47;
        const uint8_t GYRO_ZOUT_L     = 0x48;
        const uint8_t EXT_SENS_DATA_00  = 0x49;
        const uint8_t EXT_SENS_DATA_01  = 0x4A;
        const uint8_t EXT_SENS_DATA_02  = 0x4B;
        const uint8_t EXT_SENS_DATA_03  = 0x4C;
        const uint8_t EXT_SENS_DATA_04  = 0x4D;
        const uint8_t EXT_SENS_DATA_05  = 0x4E;
        const uint8_t EXT_SENS_DATA_06  = 0x4F;
        const uint8_t EXT_SENS_DATA_07  = 0x50;
        const uint8_t EXT_SENS_DATA_08  = 0x51;
        const uint8_t EXT_SENS_DATA_09  = 0x52;
        const uint8_t EXT_SENS_DATA_10  = 0x53;
        const uint8_t EXT_SENS_DATA_11  = 0x54;
        const uint8_t EXT_SENS_DATA_12  = 0x55;
        const uint8_t EXT_SENS_DATA_13  = 0x56;
        const uint8_t EXT_SENS_DATA_14  = 0x57;
        const uint8_t EXT_SENS_DATA_15  = 0x58;
        const uint8_t EXT_SENS_DATA_16  = 0x59;
        const uint8_t EXT_SENS_DATA_17  = 0x5A;
        const uint8_t EXT_SENS_DATA_18  = 0x5B;
        const uint8_t EXT_SENS_DATA_19  = 0x5C;
        const uint8_t EXT_SENS_DATA_20  = 0x5D;
        const uint8_t EXT_SENS_DATA_21  = 0x5E;
        const uint8_t EXT_SENS_DATA_22  = 0x5F;
        const uint8_t EXT_SENS_DATA_23  = 0x60;
        const uint8_t MOT_DETECT_STATUS = 0x61;
        const uint8_t I2C_SLV0_DO       = 0x63;
        const uint8_t I2C_SLV1_DO       = 0x64;
        const uint8_t I2C_SLV2_DO       = 0x65;
        const uint8_t I2C_SLV3_DO       = 0x66;
        const uint8_t I2C_MST_DELAY_CTRL= 0x67;
        const uint8_t SIGNAL_PATH_RESET = 0x68;
        const uint8_t MOT_DETECT_CTRL   = 0x69;
        const uint8_t USER_CTRL         = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP
        const uint8_t PWR_MGMT_1        = 0x6B; // Device defaults to the SLEEP mode
        const uint8_t PWR_MGMT_2        = 0x6C;
        const uint8_t DMP_BANK          = 0x6D;  // Activates a specific bank in the DMP
        const uint8_t DMP_RW_PNT        = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank
        const uint8_t DMP_REG           = 0x6F;  // Register in DMP from which to read or to which to write
        const uint8_t DMP_REG_1         = 0x70;
        const uint8_t DMP_REG_2         = 0x71;
        const uint8_t FIFO_COUNTH       = 0x72;
        const uint8_t FIFO_COUNTL       = 0x73;
        const uint8_t FIFO_R_W          = 0x74;
        const uint8_t WHO_AM_I          = 0x75; // Should return 0x71
        const uint8_t XA_OFFSET_H       = 0x77;
        const uint8_t XA_OFFSET_L       = 0x78;
        const uint8_t YA_OFFSET_H     = 0x7A;
        const uint8_t YA_OFFSET_L     = 0x7B;
        const uint8_t ZA_OFFSET_H     = 0x7D;
        const uint8_t ZA_OFFSET_L     = 0x7E;

        // fixme: from original leftover
        const uint8_t PWR_MGMNT_1 = 0x6B;
        const uint8_t PWR_MGMNT_2 = 0x6C;
        const uint8_t SMPDIV = 0x19;

        const uint8_t ACCEL_OUT = 0x3B;
        const uint8_t GYRO_OUT = 0x43;
        const uint8_t TEMP_OUT = 0x41;

        const uint8_t ACCEL_FS_SEL_2G = 0x00;
        const uint8_t ACCEL_FS_SEL_4G = 0x08;
        const uint8_t ACCEL_FS_SEL_8G = 0x10;
        const uint8_t ACCEL_FS_SEL_16G = 0x18;


        const uint8_t GYRO_FS_SEL_250DPS = 0x00;
        const uint8_t GYRO_FS_SEL_500DPS = 0x08;
        const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
        const uint8_t GYRO_FS_SEL_2000DPS = 0x18;

        const uint8_t ACCEL_DLPF_184 = 0x01;
        const uint8_t ACCEL_DLPF_92 = 0x02;
        const uint8_t ACCEL_DLPF_41 = 0x03;
        const uint8_t ACCEL_DLPF_20 = 0x04;
        const uint8_t ACCEL_DLPF_10 = 0x05;
        const uint8_t ACCEL_DLPF_5 = 0x06;

        const uint8_t GYRO_DLPF_184 = 0x01;
        const uint8_t GYRO_DLPF_92 = 0x02;
        const uint8_t GYRO_DLPF_41 = 0x03;
        const uint8_t GYRO_DLPF_20 = 0x04;
        const uint8_t GYRO_DLPF_10 = 0x05;
        const uint8_t GYRO_DLPF_5 = 0x06;

        const uint8_t INT_DISABLE = 0x00;
        const uint8_t INT_PULSE_50US = 0x00;
        const uint8_t INT_RAW_RDY_EN = 0x01;

        const uint8_t PWR_RESET = 0x80;
        const uint8_t CLOCK_SEL_PLL = 0x01;

        const uint8_t SEN_ENABLE = 0x00;

        const uint8_t I2C_MST_EN = 0x20;
        const uint8_t I2C_MST_CLK = 0x0D;
        const uint8_t I2C_SLV0_EN = 0x80;
        const uint8_t I2C_READ_FLAG = 0x80;

        // AK8963 registers
        const uint8_t AK8963_I2C_ADDR = 0x0C;

        const uint8_t AK8963_WHO_AM_I = 0x00;
        const uint8_t AK8963_INFO     = 0x01;
        const uint8_t AK8963_ST1      = 0x02;  // data ready status bit 0
        const uint8_t AK8963_HXL    = 0x03; // data
        const uint8_t AK8963_HXH    =0x04;
        const uint8_t AK8963_HYL    =0x05;
        const uint8_t AK8963_HYH    =0x06;
        const uint8_t AK8963_HZL    =0x07;
        const uint8_t AK8963_HZH    =0x08;
        const uint8_t AK8963_ST2      = 0x09;  // Data overflow bit 3 and data read error status bit 2
        const uint8_t AK8963_CNTL     = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
        const uint8_t AK8963_RSV = 0x0B;
        const uint8_t AK8963_ASTC = 0x0C;  // Self test control
        const uint8_t AK8963_TS1 = 0x0D;
        const uint8_t AK8963_TS2        = 0x0E;
        const uint8_t AK8963_I2CDIS   = 0x0F;  // I2C disable
        const uint8_t AK8963_ASAX     = 0x10;  // Fuse ROM x-axis sensitivity adjustment value
        const uint8_t AK8963_ASAY     = 0x11; // Fuse ROM y-axis sensitivity adjustment value
        const uint8_t AK8963_ASAZ     = 0x12;  // Fuse ROM z-axis sensitivity adjustment value


        // fixme: from original leftover
        const uint8_t AK8963_CNTL1 = 0x0A;
        const uint8_t AK8963_PWR_DOWN = 0x00;
        const uint8_t AK8963_CNT_MEAS1 = 0x12;
        const uint8_t AK8963_CNT_MEAS2 = 0x16;
        const uint8_t AK8963_FUSE_ROM = 0x0F;

        const uint8_t AK8963_CNTL2 = 0x0B;
        const uint8_t AK8963_RESET = 0x01;

        const uint8_t AK8963_ASA = 0x10;

        // may: extras
        // Using the MPU9250Teensy 3.1 Add-On shield, ADO is set to 0 
        // Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
        //#define ADO 0
        #if ADO
        const uint8_t MPU9250_ADDRESS = 0x69;  // Device address when ADO = 1
        const uint8_t AK8963_ADDRESS = 0x0C;   //  Address of magnetometer
        const uint8_t MS5637_ADDRESS = 0x76;   // Address of altimeter
        #else
        const uint8_t MPU9250_ADDRESS = 0x68;  // Device address when ADO = 0
        const uint8_t AK8963_ADDRESS = 0x0C;   //  Address of magnetometer
        const uint8_t MS5637_ADDRESS = 0x76;   // Address of altimeter
        #endif  

        // transformation matrix
        /* transform the accel and gyro axes to match the magnetometer axes */
        const int16_t tX[3] = {0,  1,  0}; 
        const int16_t tY[3] = {1,  0,  0};
        const int16_t tZ[3] = {0,  0, -1};

        bool writeRegister(uint8_t subAddress, uint8_t data);
        void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
        bool writeAK8963Register(uint8_t subAddress, uint8_t data);
        void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
        uint8_t whoAmI();
        uint8_t whoAmIAK8963();
};

#endif
