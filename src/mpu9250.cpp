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
    _address = address;
    _bus = bus;
    _userDefI2C = false;
    _useSPI = false;
}

/* MPU9250 object, input the I2C address, I2C bus, and I2C pins */
mpu9250::mpu9250(uint8_t address, uint8_t bus, i2c_pins pins){
    _address = address;
    _bus = bus;
    _pins = pins;
    _pullups = I2C_PULLUP_EXT;
    _userDefI2C = true;
    _useSPI = false;
}

/* MPU9250 object, input the I2C address, I2C bus, I2C pins, and I2C pullups */
mpu9250::mpu9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups){
    _address = address;
    _bus = bus;
    _pins = pins;
    _pullups = pullups;
    _userDefI2C = true;
    _useSPI = false;
}

/* MPU9250 object, input the SPI CS Pin */
mpu9250::mpu9250(uint8_t csPin, spi_irs mode){
    _csPin = csPin;
    _irsSPI = mode;
    _mosiPin = MOSI_PIN_11;
    _useSPI = true;
    _useSPIHS = false;
}

/* MPU9250 object, input the SPI CS Pin and MOSI Pin */
mpu9250::mpu9250(uint8_t csPin, spi_mosi_pin pin, spi_irs mode){
    _csPin = csPin;
    _mosiPin = pin;
    _irsSPI = mode;
    _useSPI = true;
    _useSPIHS = false;
}

void mpu9250::WireSetup(int intPin)
{
    // Only SPI needs device specific bus setup
    if (!_useSPI) return;

    // setting CS pin to output
    pinMode(_csPin, OUTPUT);

    // setting CS pin high
    digitalWriteFast(_csPin, HIGH);

    // Also register the intPin with SPI incase beginTransaction() is called from within an IRS
    if (_irsSPI) {
#if defined(__MK20DX128__) || defined(__MK20DX256__)

        // configure and begin the SPI
        switch( _mosiPin ) {

        case MOSI_PIN_7:    // SPI bus 0 alternate 1
        case MOSI_PIN_11:// SPI bus 0 default
            SPI.usingInterrupt(intPin);
            break;

        }

#endif

    // Teensy 3.5 || Teensy 3.6
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

        // configure and begin the SPI
        switch (_mosiPin) {

        case MOSI_PIN_0:    // SPI bus 1 default
        case MOSI_PIN_21:   // SPI bus 1 alternate
            SPI1.usingInterrupt(intPin);
            break;

        case MOSI_PIN_7:    // SPI bus 0 alternate 1
        case MOSI_PIN_11:   // SPI bus 0 default
        case MOSI_PIN_28:   // SPI bus 0 alternate 2
            SPI.usingInterrupt(intPin);
            break;

        case MOSI_PIN_44:   // SPI bus 2 default
        case MOSI_PIN_52:   // SPI bus 2 alternate
            SPI2.usingInterrupt(intPin);
            break;
        }

#endif

    // Teensy LC
#if defined(__MKL26Z64__)

        // configure and begin the SPI
        switch( _mosiPin ) {

        case MOSI_PIN_0:    // SPI bus 1 default
        case MOSI_PIN_21:   // SPI bus 1 alternate
            SPI1.usingInterrupt(intPin);
            break;

        case MOSI_PIN_7:    // SPI bus 0 alternate 1
        case MOSI_PIN_11:   // SPI bus 0 default
            SPI.usingInterrupt(intPin);
            break;

        }

#endif

    }
}

void mpu9250::WireBegin()
{
    // using SPI for communication
    if (_useSPI) {

        // Teensy 3.0 || Teensy 3.1/3.2
#if defined(__MK20DX128__) || defined(__MK20DX256__)

        // configure and begin the SPI
        switch( _mosiPin ) {

        case MOSI_PIN_7:    // SPI bus 0 alternate 1
            SPI.setMOSI(7);
            SPI.setMISO(8);
            SPI.setSCK(14);
            SPI.begin();
            break;

        case MOSI_PIN_11:// SPI bus 0 default
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
        switch (_mosiPin) {

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
        switch( _mosiPin ) {

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

    } else {
        // using I2C for communication
        if (!_userDefI2C) {
            // setup the I2C pins and pullups based on bus number if not defined by user
            // setting the I2C pins, pullups, and protecting against _bus out of range

            _pullups = I2C_PULLUP_EXT; // default to external pullups

#if defined(__MK20DX128__) // Teensy 3.0
            _pins = I2C_PINS_18_19;
            _bus = 0;
#endif

#if defined(__MK20DX256__) // Teensy 3.1/3.2
            if (_bus == 1) {
                _pins = I2C_PINS_29_30;
            } else {
                _pins = I2C_PINS_18_19;
                _bus = 0;
            }

#endif

#if defined(__MK64FX512__) // Teensy 3.5
            if (_bus == 2) {
                _pins = I2C_PINS_3_4;
            }
            else if (_bus == 1) {
                _pins = I2C_PINS_37_38;
            } else {
                _pins = I2C_PINS_18_19;
                _bus = 0;
            }

#endif

#if defined(__MK66FX1M0__) // Teensy 3.6
            if (_bus == 3) {
                _pins = I2C_PINS_56_57;
            } else if (_bus == 2) {
                _pins = I2C_PINS_3_4;
            } else if (_bus == 1) {
                _pins = I2C_PINS_37_38;
            } else {
                _pins = I2C_PINS_18_19;
                _bus = 0;
            }

#endif

#if defined(__MKL26Z64__) // Teensy LC
            if (_bus == 1) {
                _pins = I2C_PINS_22_23;
            } else {
                _pins = I2C_PINS_18_19;
                _bus = 0;
            }

#endif
        }
        // starting the I2C bus
        i2c_t3(_bus).begin(I2C_MASTER, 0x00, _pins, _pullups, I2C_RATE);
    }
}

int mpu9250::NewMagData()
{
    uint8_t rawData;
    if (_useSPI) {
        ReadRegisters(_address, EXT_SENS_DATA_00, 1, &rawData);
        return _newMagData = (rawData & 0x01);
    } else {
        return _newMagData = (ReadRegister(AK8963_ADDRESS, AK8963_ST1) & 0x01);
    }
}

int mpu9250::NewData()
{
    if (_newData) {
        _newData = 0;
        return 1;
    }
    return 0;
}

void mpu9250::GetAllCounts(int16_t *counts_out)
{
    uint8_t rawData[22];

    if (_useSPI) {
        ReadRegisters(_address, ACCEL_XOUT_H, sizeof(rawData), &rawData[0]);
    } else {
        ReadRequested(rawData);
    }

    // Turn the MSB and LSB into a signed 16-bit value
    counts_out[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    counts_out[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    counts_out[2] = ((int16_t)rawData[4] << 8) | rawData[5];
    counts_out[3] = ((int16_t)rawData[6] << 8) | rawData[7];
    counts_out[4] = ((int16_t)rawData[8] << 8) | rawData[9];
    counts_out[5] = ((int16_t)rawData[10] << 8) | rawData[11];
    counts_out[6] = ((int16_t)rawData[12] << 8) | rawData[13];

    if ((rawData[14] & 0x01) && !(rawData[21] & 0x08)) {
        // Mag data is ready && no mag overflow
        counts_out[7] = ((int16_t)rawData[16] << 8) | rawData[15];
        counts_out[8] = ((int16_t)rawData[18] << 8) | rawData[17];
        counts_out[9] = ((int16_t)rawData[20] << 8) | rawData[19];
    } else {
        counts_out[7] = counts_out[8] = counts_out[9] = 0;
    }

    // NOTE: In I2C master mode and INT_ANYRD_2CLEAR there is an issue with clears triggering
    // randomly before slave 0 register readout. This causes the I2C bus to stall. Using LATCH
    // mode circumvents this but requires the interrupt to be cleared manually.
    if (!_useSPI) ClearInterrupt();

    // FIXME: Shift of signed values is platform specific
}

void mpu9250::GetAllData(float *all_out, bus_hs mode)
{
    int16_t counts[10];
    float mag[3];

    _useSPIHS = mode;       // Use high speed SPI for data readout
    GetAllCounts(counts);   // Get raw ADC counts

    // Accel counts in m/s^2 accounting for direction of gravitational force (downwards)
#if 0
    // There was a bug that prevented writing accel calibration values to the
    // corresponding registers. Keep this here just incase.
    all_out[0] = (float)counts[0] * _accelScale - _accelBias[0];
    all_out[1] = (float)counts[1] * _accelScale - _accelBias[1];
    all_out[2] = (float)counts[2] * _accelScale - _accelBias[2];
#else
    all_out[0] = (float)counts[0] * _accelScale;
    all_out[1] = (float)counts[1] * _accelScale;
    all_out[2] = (float)counts[2] * _accelScale;
#endif

    // Temp counts in degrees celcius
    all_out[3] = ((float)counts[3] - _tempOffset)/_tempScale + _tempOffset;

    // Gyro counts in rad/s
    all_out[4] = (float)counts[4] * _gyroScale;
    all_out[5] = (float)counts[5] * _gyroScale;
    all_out[6] = (float)counts[6] * _gyroScale;

    // Return if mag data not ready or mag overflow (and use previous data)
    if (!(counts[7] | counts[8] | counts[9])) return;

    // Convert counts to microTesla, also include factory calibration per data sheet
    // and user environmental corrections for soft/hard iron distortions
    mag[0] = (float)counts[7] * _magScale * _magScale_factory[0] - _magHardIron[0];
    mag[1] = (float)counts[8] * _magScale * _magScale_factory[1] - _magHardIron[1];
    mag[2] = (float)counts[9] * _magScale * _magScale_factory[2] - _magHardIron[2];
    mag[0] *= _magSoftIron[0];
    mag[1] *= _magSoftIron[1];
    mag[2] *= _magSoftIron[2];

    // Transform magnetometer axes to match gyro/accel axes (less compute load than gyro/accel to mag)
    all_out[7] = tX[0]*mag[0] + tX[1]*mag[1] + tX[2]*mag[2];
    all_out[8] = tY[0]*mag[0] + tY[1]*mag[1] + tY[2]*mag[2];
    all_out[9] = tZ[0]*mag[0] + tZ[1]*mag[1] + tZ[2]*mag[2];

#ifdef MAG_EXPORT
    Serial.print((int)((float)counts[7] * _magScale) ); Serial.print("\t");
    Serial.print((int)((float)counts[8] * _magScale) ); Serial.print("\t");
    Serial.print((int)((float)counts[9] * _magScale) ); Serial.print("\t");
    Serial.print((int) all_out[7] ); Serial.print("\t");
    Serial.print((int) all_out[8] ); Serial.print("\t");
    Serial.print((int) all_out[9] ); Serial.print("\n");
#endif /* MAG_EXPORT */
}

void mpu9250::GetMPU9250Counts(int16_t *counts_out)
{
    uint8_t rawData[14];

    ReadRegisters(_address, ACCEL_XOUT_H, 14, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    counts_out[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    counts_out[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    counts_out[2] = ((int16_t)rawData[4] << 8) | rawData[5];
    counts_out[3] = ((int16_t)rawData[6] << 8) | rawData[7];
    counts_out[4] = ((int16_t)rawData[8] << 8) | rawData[9];
    counts_out[5] = ((int16_t)rawData[10] << 8) | rawData[11];
    counts_out[6] = ((int16_t)rawData[12] << 8) | rawData[13];

    // FIXME: Shift outcome of signed values is platform specific
}

void mpu9250::GetMPU9250Data(float *data_out, bus_hs mode)
{
    int16_t mpu9250Counts[7];

    _useSPIHS = mode;                   // Use high speed SPI for data readout
    GetMPU9250Counts(mpu9250Counts);    // Get raw ADC counts

    // Accel counts in g's
#if 0
    // There was a bug that prevented writing accel calibration values to the
    // corresponding registers. Keep this here just incase.
    data_out[0] = (float)mpu9250Counts[0] * _accelScale - _accelBias[0];
    data_out[1] = (float)mpu9250Counts[1] * _accelScale - _accelBias[1];
    data_out[2] = (float)mpu9250Counts[2] * _accelScale - _accelBias[2];
#else
    data_out[0] = (float)mpu9250Counts[0] * _accelScale;
    data_out[1] = (float)mpu9250Counts[1] * _accelScale;
    data_out[2] = (float)mpu9250Counts[2] * _accelScale;
#endif

    // Temp counts in degrees celcius
    data_out[3] = ((float)mpu9250Counts[3] - _tempOffset)/_tempScale + _tempOffset;

    // Gyro counts in deg/s
    data_out[4] = (float)mpu9250Counts[4] * _gyroScale;
    data_out[5] = (float)mpu9250Counts[5] * _gyroScale;
    data_out[6] = (float)mpu9250Counts[6] * _gyroScale;
}

void mpu9250::GetAccelCounts(int16_t *counts_out)
{
    uint8_t rawData[6];

    ReadRegisters(_address, ACCEL_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    counts_out[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    counts_out[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    counts_out[2] = ((int16_t)rawData[4] << 8) | rawData[5];

    // FIXME: Shift outcome of signed values is platform specific
}

void mpu9250::GetAccelData(float *accel_out, bus_hs mode)
{
    int16_t accelCounts[3];

    _useSPIHS = mode;           // Use high speed SPI for data readout
    GetGyroCounts(accelCounts); // Get raw ADC counts

    // Convert counts to g
    accel_out[0] = (float)accelCounts[0] * _accelScale;
    accel_out[1] = (float)accelCounts[1] * _accelScale;
    accel_out[2] = (float)accelCounts[2] * _accelScale;
}

void mpu9250::GetGyroCounts(int16_t *counts_out)
{
    uint8_t rawData[6];

    ReadRegisters(_address, GYRO_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    counts_out[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    counts_out[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    counts_out[2] = ((int16_t)rawData[4] << 8) | rawData[5];

    // FIXME: Shift outcome of signed values is platform specific
}

void mpu9250::GetGyroData(float *gyro_out, bus_hs mode)
{
    int16_t gyroCounts[3];

    _useSPIHS = mode;           // Use high speed SPI for data readout
    GetGyroCounts(gyroCounts);  // Get raw ADC counts

    // Convert counts to deg/s
    gyro_out[0] = (float)gyroCounts[0] * _gyroScale;
    gyro_out[1] = (float)gyroCounts[1] * _gyroScale;
    gyro_out[2] = (float)gyroCounts[2] * _gyroScale;
}

void mpu9250::GetMagCounts(int16_t *counts_out)
{
    uint8_t rawData[8];  // ST1 + x/y/z mag data + ST2 register

    if (_useSPI) {
        ReadRegisters(_address, EXT_SENS_DATA_00, sizeof(rawData), &rawData[0]);
    } else {
        ReadRegisters(AK8963_ADDRESS, AK8963_ST1, sizeof(rawData), &rawData[0]);
    }

    if ((rawData[0] & 0x01) && !(rawData[7] & 0x08)) {
        // Mag data is ready && no mag overflow
        counts_out[0] = ((int16_t)rawData[2] << 8) | rawData[1];
        counts_out[1] = ((int16_t)rawData[4] << 8) | rawData[3];
        counts_out[2] = ((int16_t)rawData[6] << 8) | rawData[5];
    }
}

void mpu9250::GetMagData(float *mag_out, bus_hs mode)
{
    int16_t magCounts[3];

    _useSPIHS = mode;           // Use high speed SPI for data readout
    GetMagCounts(magCounts);    // Get raw ADC counts

    // Convert counts to microTesla, also include factory calibration per data sheet
    // and user environmental corrections (re-scaling)
    mag_out[0] = (float)magCounts[0] * _magScale * _magScale_factory[0] - _magHardIron[0];
    mag_out[1] = (float)magCounts[1] * _magScale * _magScale_factory[1] - _magHardIron[1];
    mag_out[2] = (float)magCounts[2] * _magScale * _magScale_factory[2] - _magHardIron[2];
    mag_out[0] *= _magSoftIron[0];
    mag_out[1] *= _magSoftIron[1];
    mag_out[2] *= _magSoftIron[2];

#ifdef MAG_EXPORT
    Serial.print( (int)((float)magCounts[0] * _magScale) ); Serial.print("\t");
    Serial.print( (int)((float)magCounts[1] * _magScale) ); Serial.print("\t");
    Serial.print( (int)((float)magCounts[2] * _magScale) ); Serial.print("\t");
    Serial.print( (int) mag_out[0] ); Serial.print("\t");
    Serial.print( (int) mag_out[1] ); Serial.print("\t");
    Serial.print( (int) mag_out[2] ); Serial.print("\n");
#endif /* MAG_EXPORT */

}

int16_t mpu9250::GetTempCounts()
{
    uint8_t rawData[2];
    ReadRegisters(_address, TEMP_OUT_H, 2, &rawData[0]);
    return ((int16_t)rawData[0] << 8) | rawData[1] ;
}

float mpu9250::GetTempData(bus_hs mode)
{
    _useSPIHS = mode;   // Use high speed SPI for data readout

    return (((float)GetTempCounts() - _tempOffset) / _tempScale + _tempOffset);
}

void mpu9250::InitAK8963(ak8963_mag_range magRange, ak8963_mag_rate magRate, float *magScale_f_out)
{
    // Set mag axis scale factor
    switch (magRange) {

    case MAG_RANGE_14BIT:
        _magScale = 4912.0f / 8190.0f;  // counts to micro Tesla
        break;

    case MAG_RANGE_16BIT:
        _magScale = 4912.0f / 32760.0f; // counts micro Tesla
        break;
    }

    // Set mag sampling rate
    _magRate = magRate;

    // Reset magnetometer
    WriteAK8963Register(AK8963_CNTL2, AK8963_RESET);
    delay(10);

    // Power down magnetometer
    WriteAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
    delay(10);

    // Enter Fuse ROM access mode
    WriteAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM);
    delay(10);

    // Read the x-, y-, and z-axis factory calibration values and calculate magScale as per datasheet
    uint8_t rawData[8];
    ReadAK8963Registers( AK8963_ASAX, 3, &rawData[0]);
    _magScale_factory[0] = magScale_f_out[0] =  (float)(rawData[0] - 128) / 256.0f + 1.0f;
    _magScale_factory[1] = magScale_f_out[1] =  (float)(rawData[1] - 128) / 256.0f + 1.0f;
    _magScale_factory[2] = magScale_f_out[2] =  (float)(rawData[2] - 128) / 256.0f + 1.0f;

    // Power down magnetometer
    WriteAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
    delay(100);

    // Write magnetometer sampling rate and bit resolution to register
    uint8_t c = magRange << 4 | magRate;
    WriteAK8963Register(AK8963_CNTL1, c);
    delay(10);

    // Read one set of mag data. This tells the MPU9250 which AK8963 register(s) data has to be
    // sequentially written to the 'EXT_SENS_DATA_00' MPU9250 register. These are AK8963_ST1
    // 'magData[6]' and AK8963_ST2
    ReadAK8963Registers(AK8963_ST1, 8, &rawData[0]);
}

int mpu9250::Init(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange, uint8_t SRD)
{
    // Reset the MPU9250
    WriteRegister(_address, PWR_MGMT_1, H_RESET);

    // wait for MPU-9250 to come back up
    delay(1);

    // auto-select best clock source
    WriteRegister(_address,PWR_MGMT_1, CLKSEL_AUTO);

    // enable I2C master mode
    WriteRegister(_address,USER_CTRL, I2C_MST_EN);

    // set the I2C bus speed to 400 kHz
    WriteRegister(_address, I2C_MST_CTRL, I2C_MST_CLK);

    // delay shadow of external sensor data until rx is complete
    WriteRegister(_address, I2C_MST_DELAY_CTRL, I2C_DLY_ES_SHDW);

    // Set accel-counts to m/s^2 scale factor
    switch (accelRange) {

    case ACCEL_RANGE_2G:
        _accelScale = 2.0f / 32767.5f * _G;
        break;

    case ACCEL_RANGE_4G:
        _accelScale = 4.0f / 32767.5f * _G;
        break;

    case ACCEL_RANGE_8G:
        _accelScale = 8.0f / 32767.5f * _G;
        break;

    case ACCEL_RANGE_16G:
        _accelScale = 16.0f / 32767.5f * _G;
        break;
    }
    // By convention gravity-vector is defined positive up (+1G). For NED world coordinates the
    // z axis points downwards in the direction of the gravitational force. The accel g-vector
    // measured by the sensor has to be inverted to also point downwards:
    _accelScale *= -1;

    // Set gyro-counts to deg/s (rad/s) scale factor
    switch (gyroRange) {

    case GYRO_RANGE_250DPS:
        _gyroScale = 250.0f / 32767.5f * _d2r;
        break;

    case GYRO_RANGE_500DPS:
        _gyroScale = 500.0f / 32767.5f * _d2r;
        break;

    case GYRO_RANGE_1000DPS:
        _gyroScale = 1000.0f / 32767.5f * _d2r;
        break;

    case GYRO_RANGE_2000DPS:
        _gyroScale = 2000.0f / 32767.5f * _d2r;
        break;
    }

    // Write gyroscope full-scale range config to register
    uint8_t c = ReadRegister(_address, GYRO_CONFIG);
    c = c & ~0xE0;              // Clear self-test bits [7:5]
    c = c & ~0x18;              // Clear GFS bits [4:3]
    c = c & ~0x03;              // Clear FCHOICE bits [1:0]
    c = c | gyroRange << 3;     // Set choosen gyroRange
    WriteRegister(_address, GYRO_CONFIG, c );

    // Write accelerometer full-scale range config to register
    c = ReadRegister(_address, ACCEL_CONFIG);
    c = c & ~0xE0;              // Clear self-test bits [7:5]
    c = c & ~0x18;              // Clear AFS bits [4:3]
    c = c | accelRange << 3;    // Set choosen accelRange
    WriteRegister(_address, ACCEL_CONFIG, c);

    // Set gyro and thermometer sampling rate and digital lp-filter config
    // TODO: Allow the user to specify rate + bw
    c = ReadRegister(_address, CONFIG);
    c = c & ~0x03;          // Clear DLPFG bits [2:0]
    c = c | 0x03;           // Set gyro rate to 1kHz and bandwidth to 41Hz
    WriteRegister(_address, CONFIG, c);

    // Set accelerometer sampling rate and digital lp-filter config
    // TODO: Allow the user to specify rate + bw
    c = ReadRegister(_address, ACCEL_CONFIG2);
    c = c & ~0x0F; // Clear ACCEL_FCHOICE_B (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accel rate to 1 kHz and bandwidth to 41 Hz
    WriteRegister(_address, ACCEL_CONFIG2, c);

    // Set sensor data output rate = sampling rate/(1 + SMPLRT_DIV)
    WriteRegister(_address, SMPLRT_DIV, SRD);

    return 0;
}

void mpu9250::SetupInterrupt()
{
    if (_useSPI) {
        // Config interrupt: Auto-clear on reg read
        WriteRegister(_address, INT_PIN_CFG, INT_ANYRD_2CLEAR);
    } else {
        // NOTE: In I2C master mode and INT_ANYRD_2CLEAR there is an issue with clears triggering
        // randomly before slave 0 register readout. This causes the I2C bus to stall. Use LATCH
        // mode and clear the interrupt manually to circumvent. This sadly adds about 125us to IRS.
        WriteRegister(_address, INT_PIN_CFG, INT_LATCH_EN);
    }
}

void mpu9250::EnableInterrupt()
{
    // Trigger interrupt on new raw data
    WriteRegister(_address, INT_ENABLE, INT_RAW_RDY_EN);
    delay(100);
}

uint8_t mpu9250::ClearInterrupt()
{
    // If !INT_ANYRD_2CLEAR a 'manual' read of INT_STATUS clears the interrupt
    return ReadRegister(_address, INT_STATUS);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void mpu9250::AcelGyroCal(float *accelBias_out, float *gyroBias_out)
{
    uint8_t data[12];
    uint16_t packet_count, fifo_count;
    int32_t gyro_bias[3]  = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

    // reset device
    WriteRegister(_address, PWR_MGMT_1, H_RESET);
    delay(100);

    // Get a few (stable) sensor samples
    WriteRegister(_address, PWR_MGMT_1, CLKSEL_AUTO);
    WriteRegister(_address, PWR_MGMT_2, SENS_EN);       // enable gyro/accel x,y,z axes
    delay(200);

    // Configure device for bias calculation
    WriteRegister(_address, INT_ENABLE, 0x00);          // Disable all interrupts
    WriteRegister(_address, FIFO_EN, 0x00);             // Disable FIFO
    WriteRegister(_address, PWR_MGMT_1, CLKSEL_INTRNL); // Turn on internal clock source
    WriteRegister(_address, I2C_MST_CTRL, 0x00);        // Disable I2C master
    WriteRegister(_address, USER_CTRL, 0x00);           // Disable FIFO and I2C master modes
    WriteRegister(_address, USER_CTRL, FIFO_DMP_RST);   // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    WriteRegister(_address, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
    WriteRegister(_address, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    WriteRegister(_address, GYRO_CONFIG, 0x00);  // Set gyro FS to 250 deg/s, maximum sensitivity
    WriteRegister(_address, ACCEL_CONFIG, 0x00); // Set accel FS to 2 g, maximum sensitivity

    float gyro_res  = GFSF_250DPS;  // 131 LSB/degrees/sec
    float accel_res = AFSF_2G;      // 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    WriteRegister(_address, USER_CTRL, 0x40);   // Enable FIFO
    WriteRegister(_address, FIFO_EN, 0x78);     // Write gyro/accel to FIFO (max size 512 bytes)

    // accumulate 40 samples in 40 ms = 480 bytes
    delay(40);

    // Stop and turn off FIFO
    WriteRegister(_address, FIFO_EN, 0x00);             // Disable gyro and accel write to FIFO

    ReadRegisters(_address, FIFO_COUNTH, 2, &data[0]);  // Read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;                       // 6x (2x bytes per sensor) = 12 bytes

    // Sum data for averaging
    for (unsigned int i = 0; i < packet_count; i++) {
        int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
        ReadRegisters(_address, FIFO_R_W, sizeof(data), &data[0]);
        accel_temp[0] = (int16_t) (((uint16_t)data[0] << 8) | data[1]  );
        accel_temp[1] = (int16_t) (((uint16_t)data[2] << 8) | data[3]  );
        accel_temp[2] = (int16_t) (((uint16_t)data[4] << 8) | data[5]  );
        gyro_temp[0]  = (int16_t) (((uint16_t)data[6] << 8) | data[7]  );
        gyro_temp[1]  = (int16_t) (((uint16_t)data[8] << 8) | data[9]  );
        gyro_temp[2]  = (int16_t) (((uint16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_temp[0];
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0]  += (int32_t)gyro_temp[0];
        gyro_bias[1]  += (int32_t)gyro_temp[1];
        gyro_bias[2]  += (int32_t)gyro_temp[2];
    }

    // Get the average (biases)
    accel_bias[0] /= (int32_t)packet_count;
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0]  /= (int32_t)packet_count;
    gyro_bias[1]  /= (int32_t)packet_count;
    gyro_bias[2]  /= (int32_t)packet_count;

    if (accel_bias[2] > 0L) {
        // Remove gravity from the z-axis accelerometer bias calculation
        accel_bias[2] -= (int32_t)accel_res;
    } else {
        accel_bias[2] += (int32_t)accel_res;
    }

    // Split gyro biases into 8_BIT_H and 8_BIT_L. Biases are additive change sign accordingly
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
    data[1] = (-gyro_bias[0]/4)       & 0xFF;
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    WriteRegister(_address, XG_OFFSET_H, data[0]);
    WriteRegister(_address, XG_OFFSET_L, data[1]);
    WriteRegister(_address, YG_OFFSET_H, data[2]);
    WriteRegister(_address, YG_OFFSET_L, data[3]);
    WriteRegister(_address, ZG_OFFSET_H, data[4]);
    WriteRegister(_address, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    gyroBias_out[0] = (float)gyro_bias[0] / (float)gyro_res;
    gyroBias_out[1] = (float)gyro_bias[1] / (float)gyro_res;
    gyroBias_out[2] = (float)gyro_bias[2] / (float)gyro_res;

    // Output scaled accelerometer biases for display in the main program
    accelBias_out[0] = (float)accel_bias[0] / (float)accel_res;
    accelBias_out[1] = (float)accel_bias[1] / (float)accel_res;
    accelBias_out[2] = (float)accel_bias[2] / (float)accel_res;

#if 0
    Serial.println("accel biases (mg)");
    Serial.println(1000.0f * accelBias_out[0]);
    Serial.println(1000.0f * accelBias_out[1]);
    Serial.println(1000.0f * accelBias_out[2]);
    Serial.println("gyro biases (dps)");
    Serial.println(gyroBias_out[0]);
    Serial.println(gyroBias_out[1]);
    Serial.println(gyroBias_out[2]);
#endif

    // The MPU9250 ships with accelermometer factory trim values, the respective register values
    // can be combined with user calcualted values to imporve accuracy

    int16_t accel_bias_reg[3] = { 0, 0, 0 };
    uint8_t mask_bit[3] = { 0, 0, 0 };

    // Read factory accel trim value bits (14:7|6:1) (15bit number!), save bit 0 to 'mask_bit'
    ReadRegisters(_address, XA_OFFSET_H, 2, &data[0]);
    mask_bit[0] = data[1] & 0x01;
    accel_bias_reg[0] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE));
    ReadRegisters(_address, YA_OFFSET_H, 2, &data[0]);
    mask_bit[1] = data[1] & 0x01;
    accel_bias_reg[1] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE));
    ReadRegisters(_address, ZA_OFFSET_H, 2, &data[0]);
    mask_bit[2] = data[1] & 0x01;
    accel_bias_reg[2] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE) );

    // Factory trim value is stored in bits (14:7|6:1) of (int16_t)accel_bias_reg
    // divide by factor 2 to rightshift
    accel_bias_reg[0] /= 2;
    accel_bias_reg[1] /= 2;
    accel_bias_reg[2] /= 2;

#if 0
    Serial.println("accel factory trim (g)");
    for (int i = 0; i < 3; i++) {
        Serial.println(accel_bias_reg[i] * 16.0f / 16384.0f, 8);
    }
#endif

    // accel_bias is AFS_2G = 2/32768 = 0,06mg/LSB, div by 16 to convert to 16/16384 = 0,98mg/LSB
    accel_bias_reg[0] -= (int16_t)(accel_bias[0] / 16);
    accel_bias_reg[1] -= (int16_t)(accel_bias[1] / 16);
    accel_bias_reg[2] -= (int16_t)(accel_bias[2] / 16);

    // Split accel biases into 8_BIT_H and 8_BIT_L
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
    WriteRegister(_address, XA_OFFSET_H, data[0]);
    WriteRegister(_address, XA_OFFSET_L, data[1]);
    WriteRegister(_address, YA_OFFSET_H, data[2]);
    WriteRegister(_address, YA_OFFSET_L, data[3]);
    WriteRegister(_address, ZA_OFFSET_H, data[4]);
    WriteRegister(_address, ZA_OFFSET_L, data[5]);

#if 0
    ReadRegisters(_address, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    mask_bit[0] = data[1] & 0x01;
    accel_bias_reg[0] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE));
    ReadRegisters(_address, YA_OFFSET_H, 2, &data[0]);
    mask_bit[1] = data[1] & 0x01;
    accel_bias_reg[1] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE));
    ReadRegisters(_address, ZA_OFFSET_H, 2, &data[0]);
    mask_bit[2] = data[1] & 0x01;
    accel_bias_reg[2] = (int16_t) (((uint16_t)data[0] << 8) | (data[1] & 0xFE));

    // Factory trim value is 15bit stored in bits (14:7|6:1) of (int16_t)accel_bias_reg
    accel_bias_reg[0] /= 2;
    accel_bias_reg[1] /= 2;
    accel_bias_reg[2] /= 2;

    Serial.println("accel total bias (g)");
    for (int i = 0; i < 3; i++) {
        Serial.println(accel_bias_reg[i] * 16.0f / 16384.0f, 8);
    }
#endif
}

void mpu9250::SetMagCal(float *magBias_in, float *magScale_in)
{
    _magHardIron[0] = magBias_in[0];
    _magHardIron[1] = magBias_in[1];
    _magHardIron[2] = magBias_in[2];
    _magSoftIron[0] = magScale_in[0];
    _magSoftIron[1] = magScale_in[1];
    _magSoftIron[2] = magScale_in[2];

#if 0
    Serial.println("Preset calibration: ");
    Serial.println("AK8963 mag biases (mG)"); Serial.println(_magHardIron[0], 8);
    Serial.println(_magHardIron[1], 8); Serial.println(_magHardIron[2], 8);
    Serial.println("AK8963 mag scale (mG)"); Serial.println(_magSoftIron[0], 8);
    Serial.println(_magSoftIron[1], 8); Serial.println(_magSoftIron[2], 8);
    Serial.print("\n");
    Serial.println("Factory calibration: ");
    Serial.print("X-Axis sensitivity scale "); Serial.println(_magScale_factory[0], 2);
    Serial.print("Y-Axis sensitivity scale "); Serial.println(_magScale_factory[1], 2);
    Serial.print("Z-Axis sensitivity scale "); Serial.println(_magScale_factory[2], 2);
#endif
}

void mpu9250::MagCal(float *magBias_out, float *magScale_out)
{
    int sample_count = 0;
    int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
    int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 };
    int16_t mag_temp[3] = { 0, 0, 0 };

    // TODO: Notify the user in some way that mag cal is about to start
#if 0
    Serial.println("AK8963 initialized for active data mode....");
    Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    delay(4000);
#endif

    // shoot for ~thirty seconds of mag data
    if (_magRate == MAG_RATE_8HZ) sample_count = 30 * 8;      // at 8 Hz ODR, new mag data every 125 ms
    if (_magRate == MAG_RATE_100HZ) sample_count = 30 * 100;  // at 100 Hz ODR, new mag data every 10 ms

    for (int i = 0; i < sample_count; i++) {
        // Read the mag data
        GetMagCounts(mag_temp);

        for (int j = 0; j < 3; j++) {
            if (mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
            if (mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
        }

    if (_magRate == MAG_RATE_8HZ) delay(135);   // at 8 Hz ODR, new mag data every 125 ms
    if (_magRate == MAG_RATE_100HZ) delay(12);  // at 100 Hz ODR, new mag data every 10 ms
    }

#if 0
    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);
#endif

    // Get hard iron correction for xyz axes in counts
    // This is the offset from the sphere/elipse to the origin
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

    // save mag biases in G for main program
    _magHardIron[0] = (float)mag_bias[0] * _magScale * _magScale_factory[0];
    _magHardIron[1] = (float)mag_bias[1] * _magScale * _magScale_factory[1];
    _magHardIron[2] = (float)mag_bias[2] * _magScale * _magScale_factory[2];

    // Get soft iron correction estimate for xyz axes in counts
    // This is the sphere/elipse diameter for each dimension
    mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
    mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
    mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    // Calc circularisation factor
    _magSoftIron[0] = avg_rad / ((float)mag_scale[0]);
    _magSoftIron[1] = avg_rad / ((float)mag_scale[1]);
    _magSoftIron[2] = avg_rad / ((float)mag_scale[2]);

#if 0
    Serial.println("Mag calibration done!");
    Serial.println("AK8963 mag biases (mG)"); Serial.println(_magHardIron[0], 8);
    Serial.println(_magHardIron[1], 8); Serial.println(_magHardIron[2], 8);
    Serial.println("AK8963 mag scale (mG)"); Serial.println(_magSoftIron[0], 8);
    Serial.println(_magSoftIron[1], 8); Serial.println(_magSoftIron[2], 8);
    Serial.print("\n");
    Serial.println("Calibration values: ");
    Serial.print("X-Axis sensitivity adjustment value "); Serial.println(_magScale_factory[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(_magScale_factory[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(_magScale_factory[2], 2);
#endif
}

void mpu9250::SelfTest(float *deviation_out)
{
    uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
    uint8_t selfTest[6];
    int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] = { 0 };
    float factoryTrim[6];

    WriteRegister(_address, SMPLRT_DIV, 0x00);                      // fs divider = 0x00 + 1
    WriteRegister(_address, CONFIG, DLPF_BANDWIDTH_41HZ);           // Gyro fs = 1 kHz, filter bw = 41 Hz
    WriteRegister(_address, GYRO_CONFIG, GYRO_RANGE_250DPS << 3);   // Gyro scale to 250 dps
    WriteRegister(_address, ACCEL_CONFIG2, DLPF_BANDWIDTH_41HZ);    // Accel fs = 1 kHz, filter bw = 41 Hz
    WriteRegister(_address, ACCEL_CONFIG, ACCEL_RANGE_2G << 3);     // Accel scale to 2g

    // average 200 gyro and acclerometer values
    for (int i = 0; i < 200; i++) {

        // Read raw accel data for 8bit reg pairs and convert to int16_t
        ReadRegisters(_address, ACCEL_XOUT_H, sizeof(rawData), &rawData[0]);
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        // Read raw gyro data for 8bit reg pairs and convert to int16_t
        ReadRegisters(_address, GYRO_XOUT_H, sizeof(rawData), &rawData[0]);
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average current readings
    for (int i = 0; i < 3; i++) {
        aAvg[i] /= 200;
        gAvg[i] /= 200;
    }

    // Enable  gyro/accel self-test. Use max resolution (lowest FSCALE).
    WriteRegister(_address, ACCEL_CONFIG, SENS_ST_EN);
    WriteRegister(_address, GYRO_CONFIG,  SENS_ST_EN);
    delay(25);  // Delay a while to let the device stabilize

    // get average self-test values of gyro and acclerometer
    for (int i = 0; i < 200; i++) {

        // Read raw accel data for 8bit reg pairs and convert to int16_t
        ReadRegisters(_address, ACCEL_XOUT_H, sizeof(rawData), &rawData[0]);
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        // Read raw gyro data for 8bit reg pairs and convert to int16_t
        ReadRegisters(_address, GYRO_XOUT_H, sizeof(rawData), &rawData[0]);
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average self-test readings
    for (int i = 0; i < 3; i++) {
        aSTAvg[i] /= 200;
        gSTAvg[i] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    WriteRegister(_address, ACCEL_CONFIG, 0x00);
    WriteRegister(_address, GYRO_CONFIG,  0x00);
    delay(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test result from USR_Reg
    ReadRegisters(_address, SELF_TEST_X_ACCEL, 3, &selfTest[0]);
    ReadRegisters(_address, SELF_TEST_X_GYRO,  3, &selfTest[3]);

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620 / 1 << ACCEL_RANGE_2G) * (pow(1.01, ((float)selfTest[0] - 1.0)));
    factoryTrim[1] = (float)(2620 / 1 << ACCEL_RANGE_2G) * (pow(1.01, ((float)selfTest[1] - 1.0)));
    factoryTrim[2] = (float)(2620 / 1 << ACCEL_RANGE_2G) * (pow(1.01, ((float)selfTest[2] - 1.0)));
    factoryTrim[3] = (float)(2620 / 1 << GYRO_RANGE_250DPS) * (pow(1.01, ((float)selfTest[3] - 1.0)));
    factoryTrim[4] = (float)(2620 / 1 << GYRO_RANGE_250DPS) * (pow(1.01, ((float)selfTest[4] - 1.0)));
    factoryTrim[5] = (float)(2620 / 1 << GYRO_RANGE_250DPS) * (pow(1.01, ((float)selfTest[5] - 1.0)));

    // Report results as a ratio of (STR - FT)/FT in percent
    // TODO: Do something with this information
    for (int i = 0; i < 3; i++) {
        deviation_out[i]   = 100.0f * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.0f;
        deviation_out[i+3] = 100.0f * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i+3] - 100.0f;
    }

#if 0
    // Print the values
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(deviation_out[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(deviation_out[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(deviation_out[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(deviation_out[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(deviation_out[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(deviation_out[5],1); Serial.println("% of factory value");
#endif
}

/* Writes a register on the MPU9250 given a register address and data via SPI/I2C */
bool mpu9250::WriteRegister(uint8_t address, uint8_t subAddress, uint8_t data_in)
{
    uint8_t buff[1];

    if (_useSPI) {

// Teensy 3.0 || Teensy 3.1/3.2
#if defined(__MK20DX128__) || defined(__MK20DX256__)

        if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)) {
            // begin the transaction
            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            digitalWriteFast(_csPin,LOW);   // select the MPU9250 chip
            SPI.transfer(subAddress);       // write the register address
            SPI.transfer(data_in);          // write the data
            digitalWriteFast(_csPin,HIGH);  // deselect the MPU9250 chip
            SPI.endTransaction();           // end the transaction
        }

#endif

// Teensy 3.5 || Teensy 3.6
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

        if ((_mosiPin == MOSI_PIN_11) || (_mosiPin == MOSI_PIN_7) || (_mosiPin == MOSI_PIN_28)) {
            // begin the transaction
            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            digitalWriteFast(_csPin, LOW);  // select the MPU9250 chip
            SPI.transfer(subAddress);       // write the register address
            SPI.transfer(data_in);          // write the data
            digitalWriteFast(_csPin, HIGH); // deselect the MPU9250 chip
            SPI.endTransaction();           // end the transaction

        } else if ((_mosiPin == MOSI_PIN_0) || (_mosiPin == MOSI_PIN_21)) {
            // begin the transaction
            SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            digitalWriteFast(_csPin, LOW);  // select the MPU9250 chip
            SPI1.transfer(subAddress);      // write the register address
            SPI1.transfer(data_in);         // write the data
            digitalWriteFast(_csPin, HIGH); // deselect the MPU9250 chip
            SPI1.endTransaction();          // end the transaction

        } else if ((_mosiPin == MOSI_PIN_44) || (_mosiPin == MOSI_PIN_52)) {
            // begin the transaction
            SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            digitalWriteFast(_csPin, LOW);  // select the MPU9250 chip
            SPI2.transfer(subAddress);      // write the register address
            SPI2.transfer(data_in);         // write the data
            digitalWriteFast(_csPin, HIGH); // deselect the MPU9250 chip
            SPI2.endTransaction();          // end the transaction
        }
#endif

// Teensy LC
#if defined(__MKL26Z64__)

        if ((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)) {
            // begin the transaction
            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            digitalWriteFast(_csPin,LOW);   // select the MPU9250 chip
            SPI.transfer(subAddress);       // write the register address
            SPI.transfer(data_in);          // write the data
            digitalWriteFast(_csPin,HIGH);  // deselect the MPU9250 chip
            SPI.endTransaction();           // end the transaction

        } else if ((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)) {
            // begin the transaction
            SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            digitalWriteFast(_csPin,LOW);   // select the MPU9250 chip
            SPI1.transfer(subAddress);      // write the register address
            SPI1.transfer(data_in);         // write the data
            digitalWriteFast(_csPin,HIGH);  // deselect the MPU9250 chip
            SPI1.endTransaction();          // end the transaction
        }
#endif

    } else {
        i2c_t3(_bus).beginTransmission(address);  // Initialize the Tx buffer
        i2c_t3(_bus).write(subAddress);           // Put slave register address in Tx buffer
        i2c_t3(_bus).write(data_in);              // Put data in Tx buffer
        i2c_t3(_bus).endTransmission();           // Send the Tx buffer
    }
    delay(10); // need to slow down how fast we write to MPU9250

    /* read back the register */
    ReadRegisters(_address, subAddress, sizeof(buff), &buff[0]);

    /* check the read back register against the written register */
    if (buff[0] == data_in) {
        return true;
    } else {
        return false;
    }
}

/* Reads registers from the MPU9250 via SPI/I2C */
void mpu9250::ReadRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *data_out)
{
    if (_useSPI) {

// Teensy 3.0 || Teensy 3.1/3.2
#if defined(__MK20DX128__) || defined(__MK20DX256__)
        if ((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)) {
            // begin the transaction
            if (_useSPIHS) {
                SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            } else {
                SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin,LOW);           // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ);    // specify the starting register address

            digitalWriteFast(_csPin,HIGH);          // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW);           // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ);    // specify the starting register address

            for (uint8_t i = 0; i < count; i++) {
                data_out[i] = SPI.transfer(0x00);   // read the data
            }

            digitalWriteFast(_csPin,HIGH);  // deselect the MPU9250 chip
            SPI.endTransaction();           // end the transaction
        }
#endif

// Teensy 3.5 || Teensy 3.6
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

        if ((_mosiPin == MOSI_PIN_11) || (_mosiPin == MOSI_PIN_7) || (_mosiPin == MOSI_PIN_28)) {
            // begin the transaction
            if (_useSPIHS) {
                SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            } else {
                SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin, LOW);          // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ);    // specify the starting register address

            digitalWriteFast(_csPin, HIGH);         // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin, LOW);          // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ);    // specify the starting register address

            for (uint8_t i = 0; i < count; i++) {
                data_out[i] = SPI.transfer(0x00);   // read the data
            }

            digitalWriteFast(_csPin, HIGH);         // deselect the MPU9250 chip
            SPI.endTransaction();                   // end the transaction

        } else if ((_mosiPin == MOSI_PIN_0) || (_mosiPin == MOSI_PIN_21)) {
            // begin the transaction
            if (_useSPIHS) {
                SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            } else {
                SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin, LOW);          // select the MPU9250 chip

            SPI1.transfer(subAddress | SPI_READ);   // specify the starting register address

            digitalWriteFast(_csPin, HIGH);         // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin, LOW);          // select the MPU9250 chip

            SPI1.transfer(subAddress | SPI_READ);   // specify the starting register address

            for (uint8_t i = 0; i < count; i++) {
                data_out[i] = SPI1.transfer(0x00);  // read the data
            }

            digitalWriteFast(_csPin, HIGH);         // deselect the MPU9250 chip
            SPI1.endTransaction();                  // end the transaction

        } else if ((_mosiPin == MOSI_PIN_44) || (_mosiPin == MOSI_PIN_52)) {
            // begin the transaction
            if (_useSPIHS) {
                SPI2.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            } else {
                SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin, LOW);          // select the MPU9250 chip

            SPI2.transfer(subAddress | SPI_READ);   // specify the starting register address

            digitalWriteFast(_csPin, HIGH);         // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin, LOW);          // select the MPU9250 chip

            SPI2.transfer(subAddress | SPI_READ);   // specify the starting register address

            for (uint8_t i = 0; i < count; i++) {
                data_out[i] = SPI.transfer(0x00);   // read the data
            }

            digitalWriteFast(_csPin, HIGH);  // deselect the MPU9250 chip
            SPI2.endTransaction();           // end the transaction
        }
#endif

// Teensy LC
#if defined(__MKL26Z64__)

        if ((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)) {
            // begin the transaction
            if (_useSPIHS) {
                SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            } else {
                SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin,LOW);           // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ);    // specify the starting register address

            digitalWriteFast(_csPin,HIGH);          // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW);           // select the MPU9250 chip

            SPI.transfer(subAddress | SPI_READ);    // specify the starting register address

            for (uint8_t i = 0; i < count; i++) {
                data_out[i] = SPI.transfer(0x00);   // read the data
            }

            digitalWriteFast(_csPin,HIGH);  // deselect the MPU9250 chip
            SPI.endTransaction();           // end the transaction

        } else if ((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)) {
            // begin the transaction
            if (_useSPIHS) {
                SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
            } else {
                SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
            }
            digitalWriteFast(_csPin,LOW);           // select the MPU9250 chip

            SPI1.transfer(subAddress | SPI_READ);   // specify the starting register address

            digitalWriteFast(_csPin,HIGH);          // deselect the MPU9250 chip
            delayMicroseconds(1);
            digitalWriteFast(_csPin,LOW);           // select the MPU9250 chip

            SPI1.transfer(subAddress | SPI_READ);   // specify the starting register address

            for (uint8_t i = 0; i < count; i++) {
                data_out[i] = SPI1.transfer(0x00);  // read the data
            }

            digitalWriteFast(_csPin,HIGH);  // deselect the MPU9250 chip
            SPI1.endTransaction();          // end the transaction
        }
#endif

    } else {

        i2c_t3(_bus).beginTransmission(address);                      // Begin i2c with slave at address
        i2c_t3(_bus).write(subAddress);                               // Put slave register addr in tx buffer
        i2c_t3(_bus).endTransmission(I2C_NOSTOP);                     // Send the tx buffer, keep connection alive
        i2c_t3(_bus).sendRequest(address, (size_t) count, I2C_STOP);  // Non-blocking request of 'count' data at subaddr

        //wait
        Wire.finish();

        uint8_t i = 0;

        // Read (and empty) the rx buffer as long as there is data
        while (i2c_t3(_bus).available()) {
            data_out[i++] = i2c_t3(_bus).readByte();        // Copy i2c rx buffer to our buffer
        }
    }
}

/* Reads 1 register(s) at subAddress from MPU9250 and return the data */
uint8_t mpu9250::ReadRegister(uint8_t address, uint8_t subAddress)
{
    uint8_t data;
    ReadRegisters(address, subAddress, 1, &data);
    return data;
}

void mpu9250::RequestRegisters(uint8_t address, uint8_t subAddress, uint8_t count)
{
    i2c_t3(_bus).beginTransmission(address);                      // Begin i2c with slave at address
    i2c_t3(_bus).write(subAddress);                               // Put slave register addr in tx buffer
    i2c_t3(_bus).endTransmission(I2C_NOSTOP);                     // Send the tx buffer, keep connection alive
    i2c_t3(_bus).sendRequest(address, (size_t) count, I2C_STOP);  // Non-blocking request of 'count' data at subaddr

    //Serial.println("Regested Registers!");
}

void mpu9250::ReadRequested(uint8_t *rawData_out)
{
    uint8_t i = 0;

    //Serial.println("Reading Requested!");

    // Read (and empty) the rx buffer as long as there is data
    while (i2c_t3(_bus).available()) {
        rawData_out[i++] = i2c_t3(_bus).readByte();        // Copy i2c rx buffer to our buffer
    }
}

uint8_t mpu9250::RequestedAvailable()
{
    //Serial.println("New Available!");
    return i2c_t3(_bus).done();
}

/* Writes a register on the AK8963 given a register address and data */
bool mpu9250::WriteAK8963Register(uint8_t subAddress, uint8_t data)
{
    uint8_t count = 1;
    uint8_t buff[1];

    WriteRegister(_address, I2C_SLV0_ADDR, AK8963_ADDRESS);      // SLV0 to AK8963 & set for write
    WriteRegister(_address, I2C_SLV0_REG, subAddress);           // Set desired AK8963 sub-address
    WriteRegister(_address, I2C_SLV0_DO, data);                  // Data to be sent
    WriteRegister(_address, I2C_SLV0_CTRL, I2C_SLV0_EN | count); // Enable I2C & tx 'count' bytes

    // read the register and confirm
    ReadAK8963Registers(subAddress, sizeof(buff), &buff[0]);

    if (buff[0] == data) {
        return true;
    } else {
        return false;
    }
}

/* Reads registers from the AK8963 */
void mpu9250::ReadAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *data_out)
{
    // Config SLV0 for normal operation
    WriteRegister(_address, I2C_SLV0_ADDR, I2C_READ_FLAG | AK8963_ADDRESS); // SLV0 to AK8963 & set for read
    WriteRegister(_address, I2C_SLV0_REG, subAddress);                      // AK8963 register to read from
    WriteRegister(_address, I2C_SLV0_CTRL, I2C_SLV0_EN | count);            // Enable I2C to send 'count' bytes

    delayMicroseconds(100);     // takes some time for these registers to fill

    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    ReadRegisters(_address, EXT_SENS_DATA_00, count, data_out);
}

/* Gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
uint8_t mpu9250::whoAmI()
{
    uint8_t buff[1];

    // read the WHO AM I register
    ReadRegisters(_address, WHO_AM_I_MPU9250, sizeof(buff),&buff[0]);

    // return the register value
    return buff[0];
}

/* Gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
uint8_t mpu9250::whoAmIAK8963()
{
    uint8_t buff[1];

    // read the WHO AM I register
    ReadAK8963Registers(AK8963_WHO_AM_I, sizeof(buff),&buff[0]);

    // return the register value
    return buff[0];
}
