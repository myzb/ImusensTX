/*
 * mpu9250.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: may
 */

#include "Arduino.h"
#include "i2c_t3.h"  // I2C library
#include "SPI.h"     // SPI Library

#include "mpu9250.h"

/* MPU9250 object, input the I2C address and I2C bus */
mpu9250::mpu9250(uint8_t address, uint8_t bus){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _userDefI2C = false; // automatic I2C setup
    _useSPI = false; // set to use I2C instead of SPI
}

/* MPU9250 object, input the I2C address, I2C bus, and I2C pins */
mpu9250::mpu9250(uint8_t address, uint8_t bus, i2c_pins pins){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _pins = pins; // I2C pins
    _pullups = I2C_PULLUP_EXT; // I2C pullups
    _userDefI2C = true; // user defined I2C
    _useSPI = false; // set to use I2C instead of SPI
}

/* MPU9250 object, input the I2C address, I2C bus, I2C pins, and I2C pullups */
mpu9250::mpu9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _pins = pins; // I2C pins
    _pullups = pullups; // I2C pullups
    _userDefI2C = true; // user defined I2C
    _useSPI = false; // set to use I2C instead of SPI
}

/* MPU9250 object, input the SPI CS Pin */
mpu9250::mpu9250(uint8_t csPin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = MOSI_PIN_11; // SPI MOSI Pin, set to default
    _useSPI = true; // set to use SPI instead of I2C
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

/* MPU9250 object, input the SPI CS Pin and MOSI Pin */
mpu9250::mpu9250(uint8_t csPin, spi_mosi_pin pin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = pin; // SPI MOSI Pin
    _useSPI = true; // set to use SPI instead of I2C
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

void mpu9250::WireBegin()
{
    if( _useSPI ){ // using SPI for communication

        // setting CS pin to output
        pinMode(_csPin,OUTPUT);

        // setting CS pin high
        digitalWriteFast(_csPin,HIGH);

        // Teensy 3.0 || Teensy 3.1/3.2
        #if defined(__MK20DX128__) || defined(__MK20DX256__)

            // configure and begin the SPI
            switch( _mosiPin ){

                case MOSI_PIN_7:    // SPI bus 0 alternate 1
                    SPI.setMOSI(7);
                    SPI.setMISO(8);
                    SPI.setSCK(14);
                    SPI.begin();
                    break;
                case MOSI_PIN_11:   // SPI bus 0 default
                    SPI.setMOSI(11);
                    SPI.setMISO(12);
                    SPI.setSCK(13);
                    SPI.begin();
                    break;
            }

        #endif

        // Teensy 3.5 || Teensy 3.6
        #if defined(__MK64FX512__) || defined(__MK66FX1M0__)

            // configure and begin the SPI
            switch( _mosiPin ){

                case MOSI_PIN_0:    // SPI bus 1 default
                    SPI1.setMOSI(0);
                    SPI1.setMISO(1);
                    SPI1.setSCK(32);
                    SPI1.begin();
                    break;
                case MOSI_PIN_7:    // SPI bus 0 alternate 1
                    SPI.setMOSI(7);
                    SPI.setMISO(8);
                    SPI.setSCK(14);
                    SPI.begin();
                    break;
                case MOSI_PIN_11:   // SPI bus 0 default
                    SPI.setMOSI(11);
                    SPI.setMISO(12);
                    SPI.setSCK(13);
                    SPI.begin();
                    break;
                case MOSI_PIN_21:   // SPI bus 1 alternate
                    SPI1.setMOSI(21);
                    SPI1.setMISO(5);
                    SPI1.setSCK(20);
                    SPI1.begin();
                    break;
                case MOSI_PIN_28:   // SPI bus 0 alternate 2
                    SPI.setMOSI(28);
                    SPI.setMISO(39);
                    SPI.setSCK(27);
                    SPI.begin();
                    break;
                case MOSI_PIN_44:   // SPI bus 2 default
                    SPI2.setMOSI(44);
                    SPI2.setMISO(45);
                    SPI2.setSCK(46);
                    SPI2.begin();
                    break;
                case MOSI_PIN_52:   // SPI bus 2 alternate
                    SPI2.setMOSI(52);
                    SPI2.setMISO(51);
                    SPI2.setSCK(53);
                    SPI2.begin();
                    break;
            }

        #endif

        // Teensy LC
        #if defined(__MKL26Z64__)

            // configure and begin the SPI
            switch( _mosiPin ){

                case MOSI_PIN_0:    // SPI bus 1 default
                    SPI1.setMOSI(0);
                    SPI1.setMISO(1);
                    SPI1.setSCK(20);
                    SPI1.begin();
                    break;
                case MOSI_PIN_7:    // SPI bus 0 alternate 1
                    SPI.setMOSI(7);
                    SPI.setMISO(8);
                    SPI.setSCK(14);
                    SPI.begin();
                    break;
                case MOSI_PIN_11:   // SPI bus 0 default
                    SPI.setMOSI(11);
                    SPI.setMISO(12);
                    SPI.setSCK(13);
                    SPI.begin();
                    break;
                case MOSI_PIN_21:   // SPI bus 1 alternate
                    SPI1.setMOSI(21);
                    SPI1.setMISO(5);
                    SPI1.setSCK(20);
                    SPI1.begin();
                    break;
            }

        #endif
    }
    else{ // using I2C for communication

        if( !_userDefI2C ) { // setup the I2C pins and pullups based on bus number if not defined by user
            /* setting the I2C pins, pullups, and protecting against _bus out of range */
            _pullups = I2C_PULLUP_EXT; // default to external pullups

            #if defined(__MK20DX128__) // Teensy 3.0
                _pins = I2C_PINS_18_19;
                _bus = 0;
            #endif

            #if defined(__MK20DX256__) // Teensy 3.1/3.2
                if(_bus == 1) {
                    _pins = I2C_PINS_29_30;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif

            #if defined(__MK64FX512__) // Teensy 3.5
                if(_bus == 2) {
                    _pins = I2C_PINS_3_4;
                }
                else if(_bus == 1) {
                    _pins = I2C_PINS_37_38;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif

            #if defined(__MK66FX1M0__) // Teensy 3.6
                if(_bus == 3) {
                    _pins = I2C_PINS_56_57;
                }
                else if(_bus == 2) {
                    _pins = I2C_PINS_3_4;
                }
                else if(_bus == 1) {
                    _pins = I2C_PINS_37_38;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif

            #if defined(__MKL26Z64__) // Teensy LC
                if(_bus == 1) {
                    _pins = I2C_PINS_22_23;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif
        }

        // starting the I2C bus
        i2c_t3(_bus).begin(I2C_MASTER, 0x00, _pins, _pullups, _i2cRate);
    }
}

int mpu9250::NewMagData()
{
    return _newMagData = (ReadRegister(AK8963_ADDRESS, AK8963_ST1) & 0x01);
}

int mpu9250::NewData()
{
    if(_newData) {
        _newData = 0;
        return 1;
    }
    return 0;
}

void mpu9250::GetMPU9250Counts(int16_t *counts_out)
{
    uint8_t rawData[14];  // x/y/z accel register data stored here
    ReadRegisters(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
    counts_out[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    counts_out[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  // FIXME: Shift of signed values is platform specific
    counts_out[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
    counts_out[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
    counts_out[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
    counts_out[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
    counts_out[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void mpu9250::GetMPU9250(float *a, float *g, float &t)
{
    int16_t mpu9250Counts[7];

    // Get raw ADC counts
    GetMPU9250Counts(mpu9250Counts);

    // Accel counts in g's
#if 0
    // There was a bug that prevented writing accel calibration values to the
    // corresponding registers. Keep this here just incase.
    a[0] = (float)mpu9250Counts[0]*_accelScale - _aCal_bias[0];
    a[1] = (float)mpu9250Counts[1]*_accelScale - _aCal_bias[1];
    a[2] = (float)mpu9250Counts[2]*_accelScale - _aCal_bias[2];
#else
    a[0] = (float)mpu9250Counts[0]*_accelScale;
    a[1] = (float)mpu9250Counts[1]*_accelScale;
    a[2] = (float)mpu9250Counts[2]*_accelScale;
#endif

    // Temp in degC
    t = (float) mpu9250Counts[3] / 333.87f + 21.0f;

    // Gyro counts in deg/s
    g[0] = (float)mpu9250Counts[4]*_gyroScale;
    g[1] = (float)mpu9250Counts[5]*_gyroScale;
    g[2] = (float)mpu9250Counts[6]*_gyroScale;
}

void mpu9250::GetAccelCounts(int16_t *counts_out)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    ReadRegisters(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    counts_out[0] = ((int16_t)rawData[0] << 8) | rawData[1] ; // Turn the MSB and LSB into a signed 16-bit value
    counts_out[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    counts_out[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void mpu9250::GetAccel(float *a)
{
    int16_t accelCounts[3];

    // Get raw ADC counts
    GetGyroCounts(accelCounts);

    // Convert counts to g
    a[0] = (float)accelCounts[0]*_accelScale;
    a[1] = (float)accelCounts[1]*_accelScale;
    a[2] = (float)accelCounts[2]*_accelScale;
}

void mpu9250::GetGyroCounts(int16_t *counts_out)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    ReadRegisters(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);   // Read the six raw data registers sequentially into data array
    counts_out[0] = ((int16_t)rawData[0] << 8) | rawData[1] ; // Turn the MSB and LSB into a signed 16-bit value
    counts_out[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    counts_out[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void mpu9250::GetGyro(float *g)
{
    int16_t gyroCounts[3];

    // Get raw ADC counts
    GetGyroCounts(gyroCounts);

    // Convert counts to deg/s
    g[0] = (float)gyroCounts[0]*_gyroScale;
    g[1] = (float)gyroCounts[1]*_gyroScale;
    g[2] = (float)gyroCounts[2]*_gyroScale;
}

void mpu9250::GetMagCounts(int16_t *counts_out)
{
    uint8_t rawData[7];  // x/y/z mag register data + ST2 register
    ReadRegisters(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
    uint8_t c = rawData[6];

    // Check if magnetic sensor overflow set, if not then report data
    if(!(c & 0x08)) {
        counts_out[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
        counts_out[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
        counts_out[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
}

void mpu9250::GetMag(float *m)
{
    int16_t magCounts[3];

    // Get raw ADC counts
    GetMagCounts(magCounts);

    // Convert counts to milliGauss, also include factory calibration per data sheet
    // and user environmental corrections (re-scaling)
    m[0] = (float)magCounts[0]*_magScale*_mRes_factory[0] - _mCal_bias[0];
    m[1] = (float)magCounts[1]*_magScale*_mRes_factory[1] - _mCal_bias[1];
    m[2] = (float)magCounts[2]*_magScale*_mRes_factory[2] - _mCal_bias[2];
    m[0] *= _mCal_scale[0];
    m[1] *= _mCal_scale[1];
    m[2] *= _mCal_scale[2];

#ifdef MAG_EXPORT
    Serial.print( (int)((float)magCounts[0]*_magScale) ); Serial.print("\t");
    Serial.print( (int)((float)magCounts[1]*_magScale) ); Serial.print("\t");
    Serial.print( (int)((float)magCounts[2]*_magScale) ); Serial.print("\t");
    Serial.print( (int) m[0] ); Serial.print("\t");
    Serial.print( (int) m[1] ); Serial.print("\t");
    Serial.print( (int) m[2] ); Serial.print("\n");
#endif /* MAG_EXPORT */

}

int16_t mpu9250::GetTempCounts()
{
    uint8_t rawData[2];
    ReadRegisters(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
    return ((int16_t)rawData[0] << 8) | rawData[1] ;
}

float mpu9250::GetTemp()
{
    return (float)((GetTempCounts()  - _tempOffset)/_tempScale + _tempOffset);
}

void mpu9250::InitAK8963(ak8963_mag_range magRange, ak8963_mag_rate magRate, float *mRes_f_out)
{
    // Set mag axis scale factor
    switch (magRange) {

    case MAG_RANGE_14BIT:
        _magScale = 10.*4912./8190.0f;
        break;

    case MAG_RANGE_16BIT:
        _magScale = 10.*4912./32760.0f;
        break;
    }

    // Set mag sampling rate
    _magRate = magRate;

    // TODO: May want to reset mag sensor aswell?

    // Power down magnetometer
    WriteRegister(AK8963_ADDRESS, AK8963_CNTL, AK8963_PWR_DOWN);
    delay(10);

    // Enter Fuse ROM access mode
    WriteRegister(AK8963_ADDRESS, AK8963_CNTL, AK8963_FUSE_ROM);
    delay(10);

    // Read the x-, y-, and z-axis factory calibration values and calculate _mRes* as per datasheet
    uint8_t rawData[3];
    ReadRegisters(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);
    _mRes_factory[0] = mRes_f_out[0] =  (float)(rawData[0] - 128)/256. + 1.;
    _mRes_factory[1] = mRes_f_out[1] =  (float)(rawData[1] - 128)/256. + 1.;
    _mRes_factory[2] = mRes_f_out[2] =  (float)(rawData[2] - 128)/256. + 1.;

    // Power down magnetometer
    WriteRegister(AK8963_ADDRESS, AK8963_CNTL, AK8963_PWR_DOWN);
    delay(10);

    // Write magnetometer sampling rate and bit resolution to register
    uint8_t c = magRange << 4 | magRate;
    WriteRegister(AK8963_ADDRESS, AK8963_CNTL, c);
    delay(10);
}

void mpu9250::Init(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange, uint8_t SRD)
{
    // Wake up device and reset all registers
    WriteRegister(MPU9250_ADDRESS, PWR_MGMT_1, H_RESET);
    delay(100); // Wait for all registers to reset

    // Auto select best clock source
    WriteRegister(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    delay(200);

    // Set the accel and gyro axis scale factors
    switch(accelRange) {

        case ACCEL_RANGE_2G:
            // setting the accel range to 2G
            _accelScale = _G * 2.0f/32767.5f;
            break;

        case ACCEL_RANGE_4G:
            // setting the accel range to 4G
            _accelScale = _G * 4.0f/32767.5f;
            break;

        case ACCEL_RANGE_8G:
            // setting the accel range to 8G
            _accelScale = _G * 8.0f/32767.5f;
            break;

        case ACCEL_RANGE_16G:
            // setting the accel range to 16G
            _accelScale = _G * 16.0f/32767.5f;
            break;
    }

    switch(gyroRange) {
        case GYRO_RANGE_250DPS:
            // setting the gyro range to 250DPS
            _gyroScale = 250.0f/32767.5f * _d2r;
            break;

        case GYRO_RANGE_500DPS:
            // setting the gyro range to 500DPS
            _gyroScale = 500.0f/32767.5f * _d2r;
            break;

        case GYRO_RANGE_1000DPS:
            // setting the gyro range to 1000DPS
            _gyroScale = 1000.0f/32767.5f * _d2r;
            break;

        case GYRO_RANGE_2000DPS:
            // setting the gyro range to 2000DPS
            _gyroScale = 2000.0f/32767.5f * _d2r;
            break;
    }

    // Write gyroscope full-scale range config to register
    uint8_t c = ReadRegister(MPU9250_ADDRESS, GYRO_CONFIG);
    c = c & ~0xE0;              // Clear self-test bits [7:5]
    c = c & ~0x18;              // Clear GFS bits [4:3]
    c = c & ~0x03;              // Clear FCHOICE bits [1:0]
    c = c | gyroRange << 3;     // Set choosen gyroRange
    WriteRegister(MPU9250_ADDRESS, GYRO_CONFIG, c );

    // Write accelerometer full-scale range config to register
    c = ReadRegister(MPU9250_ADDRESS, ACCEL_CONFIG);
    c = c & ~0xE0;              // Clear self-test bits [7:5]
    c = c & ~0x18;              // Clear AFS bits [4:3]
    c = c | accelRange << 3;    // Set choosen accelRange
    WriteRegister(MPU9250_ADDRESS, ACCEL_CONFIG, c);

    // Set gyro and thermometer sampling rate and digital lp-filter config
    // TODO: Allow the user to specify rate + bw
    ReadRegister(MPU9250_ADDRESS, CONFIG);
    c = c & ~0x03;          // Clear DLPFG bits [2:0]
    c = c | 0x03;           // Set gyro rate to 1kHz and bandwidth to 41Hz
    WriteRegister(MPU9250_ADDRESS, CONFIG, c);

    // Set accelerometer sampling rate and digital lp-filter config
    // TODO: Allow the user to specify rate + bw
    c = ReadRegister(MPU9250_ADDRESS, ACCEL_CONFIG2);
    c = c & ~0x0F; // Clear ACCEL_FCHOICE_B (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accel rate to 1 kHz and bandwidth to 41 Hz
    WriteRegister(MPU9250_ADDRESS, ACCEL_CONFIG2, c);

    // Set sensor data output rate = sampling rate/(1 + SMPLRT_DIV)
    WriteRegister(MPU9250_ADDRESS, SMPLRT_DIV, SRD);

    // Config interrupt, Clear on read | i2c bypass enable
    WriteRegister(MPU9250_ADDRESS, INT_PIN_CFG, INT_ANYRD_2CLEAR | INT_BYPASS_EN);

    // Trigger interrupt on new raw data
    WriteRegister(MPU9250_ADDRESS, INT_ENABLE, INT_RAW_RDY_EN);
    delay(100);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void mpu9250::AcelGyroCal(float *accelBias_out, float *gyroBias_out)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    WriteRegister(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    WriteRegister(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    WriteRegister(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);   // enable gyro/accel x,y,z axes
    delay(200);

    // Configure device for bias calculation
    WriteRegister(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    WriteRegister(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    WriteRegister(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    WriteRegister(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    WriteRegister(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    WriteRegister(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    WriteRegister(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    WriteRegister(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    WriteRegister(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    WriteRegister(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    float  gyrosensitivity  = GFSF_250DPS;  // 131 LSB/degrees/sec
    float  accelsensitivity = AFSF_2G;      // 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    WriteRegister(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
    WriteRegister(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    WriteRegister(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    ReadRegisters(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        ReadRegisters(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t) (((uint16_t)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((uint16_t)data[2] << 8) | data[3]  );
        accel_temp[2] = (int16_t) (((uint16_t)data[4] << 8) | data[5]  );
        gyro_temp[0]  = (int16_t) (((uint16_t)data[6] << 8) | data[7]  );
        gyro_temp[1]  = (int16_t) (((uint16_t)data[8] << 8) | data[9]  );
        gyro_temp[2]  = (int16_t) (((uint16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }

    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {
        // Remove gravity from the z-axis accelerometer bias calculation
        accel_bias[2] -= (int32_t) accelsensitivity;
    } else {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    WriteRegister(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    WriteRegister(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    WriteRegister(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    WriteRegister(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    WriteRegister(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    WriteRegister(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    gyroBias_out[0] = (float)gyro_bias[0]/(float)gyrosensitivity;
    gyroBias_out[1] = (float)gyro_bias[1]/(float)gyrosensitivity;
    gyroBias_out[2] = (float)gyro_bias[2]/(float)gyrosensitivity;

    // Output scaled accelerometer biases for display in the main program
    accelBias_out[0] = (float)accel_bias[0]/(float)accelsensitivity;
    accelBias_out[1] = (float)accel_bias[1]/(float)accelsensitivity;
    accelBias_out[2] = (float)accel_bias[2]/(float)accelsensitivity;

    // The MPU9250 ships with accelermometer factory trim values, the respective register values
    // can be combined with user calcualted values to imporve accuracy

    int16_t accel_bias_reg[3] = {0, 0, 0};
    uint8_t mask_bit[3] = {0, 0, 0};

    // Read factory accel trim value bits (14:7|6:1) (15bit number!), save bit 0 to 'mask_bit'
    ReadRegisters(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);
    mask_bit[0] = data[1] & 0x01;
    accel_bias_reg[0] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE) );
    ReadRegisters(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    mask_bit[1] = data[1] & 0x01;
    accel_bias_reg[1] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE) );
    ReadRegisters(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    mask_bit[2] = data[1] & 0x01;
    accel_bias_reg[2] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE) );

    // Factory trim value is stored in bits (14:7|6:1) of (int16_t)accel_bias_reg
    // divide by factor 2 to rightshift
    accel_bias_reg[0] /= 2;
    accel_bias_reg[1] /= 2;
    accel_bias_reg[2] /= 2;

    Serial.println("accel factory trim (g)");
    for (int i = 0; i < 3; i++) {
        Serial.println(accel_bias_reg[i]*16.0f/16384.0f, 8);
    }

    // accel_bias is AFS_2G = 2/32768 = 0,06mg/LSB, div by 16 to convert to 16/16384 = 0,98mg/LSB
    accel_bias_reg[0] -= (int16_t)(accel_bias[0]/16);
    accel_bias_reg[1] -= (int16_t)(accel_bias[1]/16);
    accel_bias_reg[2] -= (int16_t)(accel_bias[2]/16);

    // Split accel_bias_reg into 8_BIT_H and 8_BIT_L
    data[0] = (accel_bias_reg[0] >> 7) & 0xFF; // bits (14:7) to XA_OFFSET_H bits (7:0)
    data[1] = (accel_bias_reg[0] << 1) & 0xFF; // bits  (6:0) to XA_OFFSET_L bits (7:1)
    data[1] = data[1] | mask_bit[0];           // restore saved  XA_OFFSET_L bit 0

    data[2] = (accel_bias_reg[1] >> 7) & 0xFF;
    data[3] = (accel_bias_reg[1] << 1) & 0xFF;
    data[3] = data[3] | mask_bit[1];

    data[4] = (accel_bias_reg[2] >> 7) & 0xFF;
    data[5] = (accel_bias_reg[2] << 1) & 0xFF;
    data[5] = data[5] | mask_bit[2];

    // Push accel biases to hardware registers
    WriteRegister(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
    WriteRegister(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
    WriteRegister(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
    WriteRegister(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
    WriteRegister(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
    WriteRegister(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
#if 0
    ReadRegisters(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    mask_bit[0] = data[1] & 0x01;
    accel_bias_reg[0] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE) );
    ReadRegisters(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    mask_bit[1] = data[1] & 0x01;
    accel_bias_reg[1] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE) );
    ReadRegisters(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    mask_bit[2] = data[1] & 0x01;
    accel_bias_reg[2] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE) );

    // Factory trim value is 15bit stored in bits (14:7|6:1) of (int16_t)accel_bias_reg
    accel_bias_reg[0] /= 2;
    accel_bias_reg[1] /= 2;
    accel_bias_reg[2] /= 2;

    Serial.println("accel total bias (g)");
    for (int i = 0; i < 3; i++) {
        Serial.println(accel_bias_reg[i]*16.0f/16384.0f, 8);
    }
#endif
}

void mpu9250::SetMagCal(float *magBias_in, float *magScale_in)
{
    _mCal_bias[0] = magBias_in[0];
    _mCal_bias[1] = magBias_in[1];
    _mCal_bias[2] = magBias_in[2];
    _mCal_scale[0] = magScale_in[0];
    _mCal_scale[1] = magScale_in[1];
    _mCal_scale[2] = magScale_in[2];
}

void mpu9250::MagCal(float *magBias_out, float *magScale_out)
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    // shoot for ~thirty seconds of mag data
    if(_magRate == MAG_RATE_8HZ) sample_count = 30*8;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(_magRate == MAG_RATE_100HZ) sample_count = 30*100;  // at 100 Hz ODR, new mag data is available every 10 ms

    for(ii = 0; ii < sample_count; ii++) {
        GetMagCounts(mag_temp);  // Read the mag data

        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }

    if(_magRate == MAG_RATE_8HZ) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(_magRate == MAG_RATE_100HZ) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    magBias_out[0] = (float) mag_bias[0]*_magScale * _mRes_factory[0];  // save mag biases in G for main program
    magBias_out[1] = (float) mag_bias[1]*_magScale * _mRes_factory[1];
    magBias_out[2] = (float) mag_bias[2]*_magScale * _mRes_factory[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    magScale_out[0] = avg_rad/((float)mag_scale[0]);
    magScale_out[1] = avg_rad/((float)mag_scale[1]);
    magScale_out[2] = avg_rad/((float)mag_scale[2]);
}

void mpu9250::SelfTest(float *selfTest_out)
{
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float factoryTrim[6];
    uint8_t FS = 0; // Max sensitivity AFS_2G and GFS_250DPS

    WriteRegister(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);       // Gyro sample rate to 1 kHz
    WriteRegister(MPU9250_ADDRESS, CONFIG, 0x02);           // Gyro sample rate to 1 kHz and DLPF to 92 Hz
    WriteRegister(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3);   // Gyro scale to 250 dps
    WriteRegister(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);    // Accel rate to 1 kHz and bandwidth to 92 Hz
    WriteRegister(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3);  // Accel scale to 2g

    // average 200 gyro and acclerometer values
    for( int ii = 0; ii < 200; ii++) {

        // Read raw accel data for 8bit reg pairs and convert to int16_t
        ReadRegisters(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        // Read raw gyro data for 8bit reg pairs and convert to int16_t
        ReadRegisters(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average current readings
    for (int ii =0; ii < 3; ii++) {
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the gyro/accel for self-test. Use max sensitivity (lowest FSCALE).
    WriteRegister(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0);
    WriteRegister(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0);
    delay(25);  // Delay a while to let the device stabilize

    for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

        // Read raw accel data for 8bit reg pairs and convert to int16_t
        ReadRegisters(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        // Read raw gyro data for 8bit reg pairs and convert to int16_t
        ReadRegisters(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average self-test readings
    for (int ii =0; ii < 3; ii++) {
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    WriteRegister(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    WriteRegister(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
    delay(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test result from USR_Reg
    selfTest[0] = ReadRegister(MPU9250_ADDRESS, SELF_TEST_X_ACCEL);
    selfTest[1] = ReadRegister(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL);
    selfTest[2] = ReadRegister(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL);
    selfTest[3] = ReadRegister(MPU9250_ADDRESS, SELF_TEST_X_GYRO);
    selfTest[4] = ReadRegister(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);
    selfTest[5] = ReadRegister(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) ));
    factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) ));
    factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) ));
    factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) ));
    factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) ));
    factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) ));

    // Report results as a ratio of (STR - FT)/FT in percent
    for (int i = 0; i < 3; i++) {
        selfTest_out[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;
        selfTest_out[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.;
    }
    // Return percent deviation from factory trim values, +/- 14 or less deviation is a pass
}

// I2C read/write functions for the MPU9250 and AK8963 sensors
void mpu9250::WriteRegister(uint8_t address, uint8_t subAddress, uint8_t data_in)
{
    /* write data to device */
    if( _useSPI ){

// Teensy 3.0 || Teensy 3.1/3.2
#if defined(__MK20DX128__) || defined(__MK20DX256__)

            if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
                SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
                SPI.transfer(subAddress); // write the register address
                SPI.transfer(data_in); // write the data
                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                SPI.endTransaction(); // end the transaction
            }

#endif

// Teensy 3.5 || Teensy 3.6
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

            if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
                SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
                SPI.transfer(subAddress); // write the register address
                SPI.transfer(data_in); // write the data
                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                SPI.endTransaction(); // end the transaction
            }
            else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
                SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
                SPI1.transfer(subAddress); // write the register address
                SPI1.transfer(data_in); // write the data
                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                SPI1.endTransaction(); // end the transaction
            }
            else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
                SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
                SPI2.transfer(subAddress); // write the register address
                SPI2.transfer(data_in); // write the data
                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                SPI2.endTransaction(); // end the transaction
            }
#endif

// Teensy LC
#if defined(__MKL26Z64__)

            if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
                SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
                SPI.transfer(subAddress); // write the register address
                SPI.transfer(data_in); // write the data
                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                SPI.endTransaction(); // end the transaction
            }
            else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
                SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
                SPI1.transfer(subAddress); // write the register address
                SPI1.transfer(data_in); // write the data
                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                SPI1.endTransaction(); // end the transaction
            }
#endif

    } else {
        i2c_t3(_bus).beginTransmission(address);  // Initialize the Tx buffer
        i2c_t3(_bus).write(subAddress);           // Put slave register address in Tx buffer
        i2c_t3(_bus).write(data_in);                 // Put data in Tx buffer
        i2c_t3(_bus).endTransmission();           // Send the Tx buffer
    }
}

uint8_t mpu9250::ReadRegister(uint8_t address, uint8_t subAddress)
{
    uint8_t data;                            // `data` will store the register data
    i2c_t3(_bus).beginTransmission(address);         // Initialize the Tx buffer
    i2c_t3(_bus).write(subAddress);                  // Put slave register address in Tx buffer
    i2c_t3(_bus).endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//  i2c_t3(_bus).endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//  i2c_t3(_bus).requestFrom(address, 1);            // Read one byte from slave register address
    i2c_t3(_bus).requestFrom(address, (size_t) 1);   // Read one byte from slave register address
    data = i2c_t3(_bus).read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void mpu9250::ReadRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *data_out)
{
    if( _useSPI ){

// Teensy 3.0 || Teensy 3.1/3.2
#if defined(__MK20DX128__) || defined(__MK20DX256__)
        if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
            // begin the transaction
            if(_useSPIHS){
                SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            else{
                SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ); // specify the starting register address

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ); // specify the starting register address

            for(uint8_t i = 0; i < count; i++){
                data_out[i] = SPI.transfer(0x00); // read the data
            }

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            SPI.endTransaction(); // end the transaction
        }
#endif

// Teensy 3.5 || Teensy 3.6
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

        if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
            // begin the transaction
            if(_useSPIHS){
                SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            else{
                SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ); // specify the starting register address

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ); // specify the starting register address

            for(uint8_t i = 0; i < count; i++){
                data_out[i] = SPI.transfer(0x00); // read the data
            }

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            SPI.endTransaction(); // end the transaction
        }
        else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
            // begin the transaction
            if(_useSPIHS){
                SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            else{
                SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

            for(uint8_t i = 0; i < count; i++){
                data_out[i] = SPI1.transfer(0x00); // read the data
            }

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            SPI1.endTransaction(); // end the transaction
        }
        else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
            // begin the transaction
            if(_useSPIHS){
                SPI2.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            else{
                SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

            for(uint8_t i = 0; i < count; i++){
                data_out[i] = SPI.transfer(0x00); // read the data
            }

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            SPI2.endTransaction(); // end the transaction
        }
#endif

// Teensy LC
#if defined(__MKL26Z64__)

        if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
            // begin the transaction
            if(_useSPIHS){
                SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            else{
                SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ); // specify the starting register address

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ); // specify the starting register address

            for(uint8_t i = 0; i < count; i++){
                data_out[i] = SPI.transfer(0x00); // read the data
            }

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            SPI.endTransaction(); // end the transaction
        }
        else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
            // begin the transaction
            if(_useSPIHS){
                SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            else{
                SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

            SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

            for(uint8_t i = 0; i < count; i++){
                data_out[i] = SPI1.transfer(0x00); // read the data
            }

            digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
            SPI1.endTransaction(); // end the transaction
        }
#endif

    } else {
        i2c_t3(_bus).beginTransmission(address);   // Initialize the Tx buffer
        i2c_t3(_bus).write(subAddress);            // Put slave register address in Tx buffer
        i2c_t3(_bus).endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
    //  i2c_t3(_bus).endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
        uint8_t i = 0;
    //        i2c_t3(_bus).requestFrom(address, count);     // Read bytes from slave register address
        i2c_t3(_bus).requestFrom(address, (size_t) count);  // Read bytes from slave register address
        while (i2c_t3(_bus).available()) {
            data_out[i++] = i2c_t3(_bus).read();       // Put read results in the Rx buffer
        }
    }
}

/* writes a register to the AK8963 given a register address and data */
bool mpu9250::WriteAK8963Register(uint8_t subAddress, uint8_t data)
{
    uint8_t count = 1;
    uint8_t buff[1];

    WriteRegister(_address, I2C_SLV0_ADDR, AK8963_ADDRESS); // set slave 0 to the AK8963 and set for write
    WriteRegister(_address, I2C_SLV0_REG, subAddress); // set the register to the desired AK8963 sub address
    WriteRegister(_address, I2C_SLV0_DO, data); // store the data for write
    WriteRegister(_address, I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and send 1 byte

    // read the register and confirm
    ReadAK8963Registers(subAddress, sizeof(buff), &buff[0]);

    if(buff[0] == data) {
        return true;
    }
    else{
        return false;
    }
}

/* reads registers from the AK8963 */
void mpu9250::ReadAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *data_out)
{
    WriteRegister(_address, I2C_SLV0_ADDR, AK8963_ADDRESS | I2C_READ_FLAG); // set slave 0 to the AK8963 and set for read
    WriteRegister(_address, I2C_SLV0_REG,subAddress); // set the register to the desired AK8963 sub address
    WriteRegister(_address, I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and request the bytes
    delayMicroseconds(100); // takes some time for these registers to fill
    ReadRegisters(_address, EXT_SENS_DATA_00, count, data_out); // read the bytes off the MPU9250 EXT_SENS_DATA registers
}
