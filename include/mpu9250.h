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

enum mpu9250_gyro_range {
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
};

enum mpu9250_accel_range {
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
};

enum mpu9250_dlpf_bandwidth {
    DLPF_BANDWIDTH_184HZ,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ
};

enum ak8963_mag_range {
    MAG_RANGE_14BIT, // 0.6 mG per LSB
    MAG_RANGE_16BIT  // 0.15 mG per LSB
};

enum ak8963_mag_rate {
    MAG_RATE_8HZ   = 0x02,
    MAG_RATE_100HZ = 0x06
};

/* The MPU9250 Class
 *
 * See also MPU-9250 Register Map and Descriptions, RM-MPU-9250A-00, Rev. 1.6, 01/07/2015
 * for registers not listed in above document; the MPU9250 and MPU9150 are virtually identical but the
 * latter has a different register map
 *
 */

class mpu9250 {

public:
    /* AK8963 Magnetometer Registers */
    const uint8_t AK8963_ADDRESS   = 0x0C;  // Magnetometer address

    const uint8_t AK8963_WHO_AM_I  = 0x00;  // Should return 0x48
    const uint8_t AK8963_INFO      = 0x01;
    const uint8_t AK8963_ST1       = 0x02;
    const uint8_t AK8963_XOUT_L    = 0x03;
    const uint8_t AK8963_XOUT_H    = 0x04;
    const uint8_t AK8963_YOUT_L    = 0x05;
    const uint8_t AK8963_YOUT_H    = 0x06;
    const uint8_t AK8963_ZOUT_L    = 0x07;
    const uint8_t AK8963_ZOUT_H    = 0x08;
    const uint8_t AK8963_ST2       = 0x09;
    const uint8_t AK8963_CNTL1     = 0x0A;
    const uint8_t AK8963_CNTL2     = 0x0B;
    const uint8_t AK8963_ASTC      = 0x0C;
    const uint8_t AK8963_I2CDIS    = 0x0F;
    const uint8_t AK8963_ASAX      = 0x10;
    const uint8_t AK8963_ASAY      = 0x11;
    const uint8_t AK8963_ASAZ      = 0x12;
    /* End */

    /* MPU 9250 Gyro/Accel Registers */
    const uint8_t MPU9250_ADDRESS   = 0x68; // MPU Address when AD0 = L
    const uint8_t MPU9250_ADDRESS_B = 0x69; // MPU Address when AD0 = H

    const uint8_t SELF_TEST_X_GYRO = 0x00;
    const uint8_t SELF_TEST_Y_GYRO = 0x01;
    const uint8_t SELF_TEST_Z_GYRO = 0x02;

    const uint8_t SELF_TEST_X_ACCEL = 0x0D;
    const uint8_t SELF_TEST_Y_ACCEL = 0x0E;
    const uint8_t SELF_TEST_Z_ACCEL = 0x0F;

    const uint8_t SELF_TEST_A       = 0x10;

    const uint8_t XG_OFFSET_H      = 0x13;
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

    const uint8_t MOT_DUR          = 0x20;
    const uint8_t ZMOT_THR         = 0x21;
    const uint8_t ZRMOT_DUR        = 0x22;

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
    const uint8_t DMP_INT_STATUS   = 0x39;
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
    const uint8_t USER_CTRL          = 0x6A;
    const uint8_t PWR_MGMT_1         = 0x6B;
    const uint8_t PWR_MGMT_2         = 0x6C;

    const uint8_t DMP_BANK         = 0x6D;
    const uint8_t DMP_RW_PNT       = 0x6E;
    const uint8_t DMP_REG          = 0x6F;
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
    /* End */

    /* Register flags */
    // MPU9250 general
    const uint8_t H_RESET          = 0x80;
    const uint8_t CLKSEL_AUTO      = 0x01;
    const uint8_t INT_ANYRD_2CLEAR = 0x10;
    const uint8_t INT_RAW_RDY_EN   = 0x01;
    const uint8_t INT_BYPASS_EN    = 0x02;
    const uint8_t INT_LATCH_EN     = 0x20;
    const uint8_t I2C_READ_FLAG    = 0x80;
    const uint8_t I2C_WAIT_FOR_ES  = 0x40;
    const uint8_t I2C_DLY_ES_SHDW  = 0x80;
    const uint8_t I2C_SLV0_GRP_EVN = 0x10;
    const uint8_t I2C_SLV0_EN      = 0x80;
    const uint8_t I2C_MST_EN       = 0x20;
    const uint8_t I2C_MST_CLK      = 0x0D;  // 400 kHz

    // AK8963 magnetometer
    const uint8_t AK8963_PWR_DOWN = 0x00;
    const uint8_t AK8963_FUSE_ROM = 0x0F;
    const uint8_t AK8963_RESET    = 0x01;
    /* End */

    /* Class constants */
    // Accel full scale factors LSB/g
    const float AFSF_2G  = 16384.0f;
    const float AFSF_4G  =  8192.0f;
    const float AFSF_8G  =  4096.0f;
    const float AFSF_16G =  2048.0f;

    // Gyro full scale factors LSB/deg/s
    const float GFSF_250DPS  = 131.0f;
    const float GFSF_500DPS  =  65.5f;
    const float GFSF_1000DPS =  32.8f;
    const float GFSF_2000DPS =  16.4f;

    // SPI/I2C
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_LS_CLOCK = 1000000;  // 1 MHz
    const uint32_t SPI_HS_CLOCK = 20000000; // 20 MHz

    // Misc
    const float _G = 9.807f;
    const float _d2r = 3.14159265359f/180.0f;
    const float _tempScale = 333.87f;
    const float _tempOffset = 21.0f;
    /* End */

    /* Member variables */
    // Sensors
    float _mCal_bias[3] = {0, 0, 0}, _mCal_scale[3]  = {0, 0, 0};   // Hard iron offset (bias), Soft iron axis re-scale
    float _mRes_factory[3] = {0, 0, 0};                             // Factory axis resolution factor
    float _gCal_bias[3] = {0, 0, 0}, _aCal_bias[3] = {0, 0, 0};     // Accel/Gyro bias

    float _accelScale, _gyroScale, _magScale;                       // Sensor resolutions per LSB
    uint8_t _magRate;                                               // Sensor sampling rate

    int _newMagData = 0;                                            // new magData flag
    int _newData = 0;                                               // new MPU9250Data flag

    // SPI/I2C
    uint8_t _address;
    uint8_t _bus;
    i2c_pins _pins;
    i2c_pullup _pullups;
    bool _userDefI2C;
    uint8_t _csPin;
    spi_mosi_pin _mosiPin;
    bool _useSPI;
    bool _useSPIHS;
    const uint32_t _i2cRate = 400000; // 400 kHz, TODO: May want to allow changing this
    /* End */

    /* Member functions */
    // Constructors
    mpu9250(uint8_t address, uint8_t bus);
    mpu9250(uint8_t address, uint8_t bus, i2c_pins pins);
    mpu9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups);
    mpu9250(uint8_t csPin);
    mpu9250(uint8_t csPin, spi_mosi_pin pin);

    // Functions that start the communication and sensors
    void WireBegin();
    int Init(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange, uint8_t SRD);
    void InitAK8963(ak8963_mag_range magRange, ak8963_mag_rate magRate, float *mRes_f_out);
    void SetupInterrupt();
    void EnableInterrupt();
    uint8_t ClearInterrupt();

    // Functions to fine tune the sensors
    void SelfTest(float *selfTest_out);
    void MagCal(float *magBias_out, float *magScale_out);
    void AcelGyroCal(float *accelBias_out, float *gyroBias_out);
    void SetMagCal(float *magBias_in, float *magScale_in);
    void GetAllData(float *all_out);

    // Functions that return the raw sensor ADC values
    int16_t GetTempCounts();
    void GetMagCounts(int16_t *counts_out);
    void GetAccelCounts(int16_t *counts_out);
    void GetGyroCounts(int16_t *counts_out);
    void GetMPU9250Counts(int16_t *counts_out);
    void GetAllCounts(int16_t *counts_out);

    int NewMagData();
    int NewData();

    // Functions that return the converted sensor values (g's, deg/s, milliGaus, DegCelcius)
    float GetTempData();
    void GetAccelData(float *accel_out);
    void GetGyroData(float *gyro_out);
    void GetMPU9250Data(float *data_out);
    void GetMagData(float *mag_out);

    // Register R/W access
    bool WriteRegister(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t ReadRegister(uint8_t address, uint8_t subAddress);
    void ReadRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *data_out);
    void ReadAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *data_out);
    bool WriteAK8963Register(uint8_t subAddress, uint8_t data);

    uint8_t whoAmI();
    uint8_t whoAmIAK8963();
    /* End */
};
#endif /* _MPU9250_H_ */
