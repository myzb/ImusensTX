/*
 * mpu9250.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: may
 *
 *  Notice:
 *    Adapted from the MPU9250 Adruino sketch by Kris Winer. Class and SPI routines were based
 *    on the implementation by Brian R Taylor from Bolder Flight Systems.
 */

#include <Arduino.h>
#include <i2c_t3.h>
#include <SPI.h>

#include "mpu9250.h"

/* MPU9250 object, input the I2C address and I2C bus */
mpu9250::mpu9250(uint8_t address, uint8_t bus)
{
    _address = address;
    _bus = bus;
    _userDefI2C = false;
    _useSPI = false;
}

/* MPU9250 object, input the I2C address, I2C bus, and I2C pins */
mpu9250::mpu9250(uint8_t address, uint8_t bus, i2c_pins pins)
{
    _address = address;
    _bus = bus;
    _pins = pins;
    _pullups = I2C_PULLUP_EXT;
    _userDefI2C = true;
    _useSPI = false;
}

/* MPU9250 object, input the I2C address, I2C bus, I2C pins, and I2C pullups */
mpu9250::mpu9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups)
{
    _address = address;
    _bus = bus;
    _pins = pins;
    _pullups = pullups;
    _userDefI2C = true;
    _useSPI = false;
}

/* MPU9250 object, input the SPI CS Pin */
mpu9250::mpu9250(uint8_t csPin)
{
    _csPin = csPin;
    _mosiPin = MOSI_PIN_11;
    _useSPI = true;
    _useSPIHS = false;
}

/* MPU9250 object, input the SPI CS Pin and MOSI Pin */
mpu9250::mpu9250(uint8_t csPin, spi_mosi_pin pin)
{
    _csPin = csPin;
    _mosiPin = pin;
    _useSPI = true;
    _useSPIHS = false;
}

void mpu9250::WireSetup()
{
    // I2C does not need extra setup
    if (!_useSPI) return;

    // each SPI device uses a unique chip select pin
    pinMode(_csPin, OUTPUT);
    digitalWriteFast(_csPin, HIGH);
}

void mpu9250::WireBegin()
{
    if (_useSPI) {
    // using SPI for communication

/* Teensy 3.x */
#if defined(KINETISK)

        // configure SPI
        switch (_mosiPin) {

        case MOSI_PIN_0:    // SPI bus 1 default
            _spiBus = &SPI1;
            _spiBus->setMOSI(0);
            _spiBus->setMISO(1);
            _spiBus->setSCK(32);
            break;

        case MOSI_PIN_7:    // SPI bus 0 alternate 1
            _spiBus = &SPI;
            _spiBus->setMOSI(7);
            _spiBus->setMISO(8);
            _spiBus->setSCK(14);
            break;

        case MOSI_PIN_11:   // SPI bus 0 default
            _spiBus = &SPI;
            _spiBus->setMOSI(11);
            _spiBus->setMISO(12);
            _spiBus->setSCK(13);
            break;

/* Teensy 3.5 || Teensy 3.6 */
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
        case MOSI_PIN_21:   // SPI bus 1 alternate
            _spiBus = &SPI1;
            _spiBus->setMOSI(21);
            _spiBus->setMISO(5);
            _spiBus->setSCK(20);
            break;

        case MOSI_PIN_28:   // SPI bus 0 alternate 2
            _spiBus = &SPI;
            _spiBus->setMOSI(28);
            _spiBus->setMISO(39);
            _spiBus->setSCK(27);
            break;

        case MOSI_PIN_44:   // SPI bus 2 default
            _spiBus = &SPI2;
            _spiBus->setMOSI(44);
            _spiBus->setMISO(45);
            _spiBus->setSCK(46);
            break;

        case MOSI_PIN_52:   // SPI bus 2 alternate
            _spiBus = &SPI2;
            _spiBus->setMOSI(52);
            _spiBus->setMISO(51);
            _spiBus->setSCK(53);
            break;
#endif /* Teensy 3.5 || Teensy 3.6 */
        }
#endif /* Teensy 3.x */

/* Teensy LC */
#if defined(__MKL26Z64__)

        // configure SPI and set _spiBus class pointer to active SPI bus instance
        switch( _mosiPin ) {

            case MOSI_PIN_0:    // SPI bus 1 default
            _spiBus = &SPI1;
            _spiBus->setMOSI(0);
            _spiBus->setMISO(1);
            _spiBus->setSCK(20);
            break;

            case MOSI_PIN_7:    // SPI bus 0 alternate 1
            _spiBus = &SPI;
            _spiBus->setMOSI(7);
            _spiBus->setMISO(8);
            _spiBus->setSCK(14);
            break;

            case MOSI_PIN_11:   // SPI bus 0 default
            _spiBus = &SPI;
            _spiBus->setMOSI(11);
            _spiBus->setMISO(12);
            _spiBus->setSCK(13);
            break;

            case MOSI_PIN_21:   // SPI bus 1 alternate
            _spiBus = &SPI1;
            _spiBus->setMOSI(21);
            _spiBus->setMISO(5);
            _spiBus->setSCK(20);
            break;
        }
#endif /* Teensy LC */

        // Start the SPI bus
        _spiBus->begin();
    } else {
        // using I2C for communication
        if (!_userDefI2C) {
            // setup the I2C pins and pullups based on bus number if not defined by user
            // setting the I2C pins, pullups, and protecting against _bus out of range

            _pullups = I2C_PULLUP_EXT; // default to external pullups

#if defined(KINETISK)
            switch (_bus) {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
            case 1:
                _pins = I2C_PINS_37_38;
                break;
#endif /* Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6*/
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
            case 2:
                _pins = I2C_PINS_3_4;
                break;
#endif /* Teensy 3.5 || Teensy 3.6 */
#if defined(__MK66FX1M0__)
            case 3:
                _pins = I2C_PINS_56_57;
                break;
#endif /* Teensy 3.6 */
            default:
                _pins = I2C_PINS_18_19;
                _bus = 0;
            }
#endif /* Teensy 3.x */

#if defined(KINETISL)
            switch (_bus) {
            case 1:
                _pins = I2C_PINS_22_23;
                break;
            default:
                _pins = I2C_PINS_18_19;
                _bus = 0;
            }
#endif /* Teensy LC */

        }
        // Set the _i2cBus class pointer to the active i2c bus instance
        switch (_bus) {
        case 1:
            _i2cBus = &Wire1;
            break;
        case 2:
            _i2cBus = &Wire2;
            break;
        case 3:
            _i2cBus = &Wire3;
            break;
        default:
            _i2cBus = &Wire;
        }

        // starting the I2C bus
        _i2cBus->begin(I2C_MASTER, 0x00, _pins, _pullups, I2C_CLK);
    }
}

void mpu9250::RequestAllData()
{
    RequestRegisters(_address, ACCEL_XOUT_H, 22);
    _requestedData = true;
}

void mpu9250::GetAllCounts(int16_t* counts_out)
{
    uint8_t rawData[22];

    if (_useSPI) {
        // Blocking call that requests and reads registers via SPI
        ReadRegisters(_address, ACCEL_XOUT_H, sizeof(rawData), &rawData[0]);
    } else {
        // I2C is non-blocking, data already was requested and is now available for read
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

void mpu9250::GetAllData(float* all_out, bus_hs mode)
{
    int16_t counts[10];
    float mag[3];

    _useSPIHS = mode;       // Use high speed SPI for data readout
    GetAllCounts(counts);   // Get raw ADC counts

    // Accel in m/s2 (ENU coords)
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
    all_out[3] = ((float)counts[3] - _tempOffset) / _tempScale + _tempOffset;

    // Gyro in rad/s (ENU coords)
    all_out[4] = (float)counts[4] * _gyroScale;
    all_out[5] = (float)counts[5] * _gyroScale;
    all_out[6] = (float)counts[6] * _gyroScale;

    // Return if mag data not ready or mag overflow
    _magReady = (counts[7] | counts[8] | counts[9]);
   if (!_magReady) return;

    // Mag in mTesla
    // Substract hardIron offset from mag counts
    mag[0] = (float)counts[7] - _magHardIron[0];
    mag[1] = (float)counts[8] - _magHardIron[1];
    mag[2] = (float)counts[9] - _magHardIron[2];

    // Apply scaling factors and convert to micro Tesla
    static float magScale_total[] = {
            _magSoftIron[0] * _magScale_factory[0]  * _magScale,
            _magSoftIron[1] * _magScale_factory[1]  * _magScale,
            _magSoftIron[2] * _magScale_factory[2]  * _magScale
    };
    mag[0] *= magScale_total[0];
    mag[1] *= magScale_total[1];
    mag[2] *= magScale_total[2];

    // Transform axes to ENU coords (east-north-up)
    all_out[7] = tX[0]*mag[0] + tX[1]*mag[1] + tX[2]*mag[2];
    all_out[8] = tY[0]*mag[0] + tY[1]*mag[1] + tY[2]*mag[2];
    all_out[9] = tZ[0]*mag[0] + tZ[1]*mag[1] + tZ[2]*mag[2];

#ifdef MAG_EXPORT
    for (int i = 7; i < 10; i++) {
        Serial.printf("%d\t", (int)((float)counts[i] * _magScale));
    }
    Serial.printf("\n");
    for (int i = 7; i < 10; i++) {
        Serial.printf("%d\t", (int) all_out[i] );
    }
#endif /* MAG_EXPORT */
}

void mpu9250::GetAll(float* all_out, bus_hs mode)
{
    int16_t counts[10];

    _useSPIHS = mode;       // Use high speed SPI for data readout
    GetAllCounts(counts);   // Get raw ADC counts

    // Accel counts (NED coords)
    all_out[0] = tX[0]*(float)counts[0] + tX[1]*(float)counts[1] + tX[2]*(float)counts[2];
    all_out[1] = tY[0]*(float)counts[0] + tY[1]*(float)counts[1] + tY[2]*(float)counts[2];
    all_out[2] = tZ[0]*(float)counts[0] + tZ[1]*(float)counts[1] + tZ[2]*(float)counts[2];

    // The 'downwards' gravitational pull on the accel probe mass causes the accel to output an
    // acceleration upwards. In NED coordinates the z-axis is positive down, therefore the upwards
    // acceleration (due to the gravitational pull) is negative. By convention +1G is a positive
    // vector that points downwards. The measured acceleration (due to gravity) has to be inverted.
    all_out[0] *= -1.0f;
    all_out[1] *= -1.0f;
    all_out[2] *= -1.0f;

    // Temp in degrees celcius
    all_out[3] = ((float)counts[3] - _tempOffset) / _tempScale + _tempOffset;

    // Gyro rad/s (NED coords)
    all_out[4] = tX[0]*(float)counts[4] + tX[1]*(float)counts[5] + tX[2]*(float)counts[6];
    all_out[5] = tY[0]*(float)counts[4] + tY[1]*(float)counts[5] + tY[2]*(float)counts[6];
    all_out[6] = tZ[0]*(float)counts[4] + tZ[1]*(float)counts[5] + tZ[2]*(float)counts[6];
    all_out[4] *= _gyroScale;
    all_out[5] *= _gyroScale;
    all_out[6] *= _gyroScale;

    // Return if mag data not ready or mag overflow
    _magReady = (counts[7] | counts[8] | counts[9]);
   if (!_magReady) return;

    // Mag counts minus hard_iron (NED coords)
    all_out[7] = (float)counts[7] - _magHardIron[0];
    all_out[8] = (float)counts[8] - _magHardIron[1];
    all_out[9] = (float)counts[9] - _magHardIron[2];
    all_out[7] *= _magScale_factory[0] *_magSoftIron[0];
    all_out[8] *= _magScale_factory[1] *_magSoftIron[1];
    all_out[9] *= _magScale_factory[2] *_magSoftIron[2];

#ifdef MAG_EXPORT
    for (int i = 7; i < 10; i++) {
        Serial.printf("%d\t", (int)((float)counts[i] * _magScale));
    }
    Serial.printf("\n");
    for (int i = 7; i < 10; i++) {
        Serial.printf("%d\t", (int) all_out[i] );
    }
#endif /* MAG_EXPORT */
}

void mpu9250::InitAK8963(ak8963_mag_range magRange, ak8963_mag_rate magRate)
{
    // Set mag axis scale factor
    switch (magRange) {

    case MAG_RANGE_14BIT:
        _magScale = 4912.0f / 8190.0f;  // counts to micro Tesla
        break;

    case MAG_RANGE_16BIT:
        _magScale = 4912.0f / 32760.0f; // counts to micro Tesla
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

    // Read the x-, y-, z-axis factory calibration values and calculate magScale as per datasheet
    uint8_t rawData[8];
    ReadAK8963Registers( AK8963_ASAX, 3, &rawData[0]);
    _magScale_factory[0] = (float)(rawData[0] - 128) / 256.0f + 1.0f;
    _magScale_factory[1] = (float)(rawData[1] - 128) / 256.0f + 1.0f;
    _magScale_factory[2] = (float)(rawData[2] - 128) / 256.0f + 1.0f;

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
    // WORKAROUND: The AK8963 magnetometer hangs in some cases when restarting the MPU after
    // reprogramming the uC. Do a quick I2C master setup to  access the AK8963 and power it down.

    // Enable internal I2C master mode
    WriteRegister(_address, USER_CTRL, I2C_MST_EN);

    // set the I2C bus speed to 400 kHz
    WriteRegister(_address, I2C_MST_CTRL, I2C_MST_CLK);

    // Power down magnetometer
    WriteAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

    // Reset the MPU9250
    WriteRegister(_address, PWR_MGMT_1, H_RESET);

    // wait for MPU9250 to come back up
    delay(10);

    // Now reset the magnetometer (again)
    WriteAK8963Register(AK8963_CNTL1, AK8963_RESET);

    // No need to again reset MPU9250
    // END WORKAROUND

    // Calibrate MPU9250 Gyroscope
    GyroCal();

    // auto-select best clock source
    WriteRegister(_address,PWR_MGMT_1, CLKSEL_AUTO);

    // Reset and enable internal I2C master mode
    if (_useSPI) {
        // Disable external I2C interface
        WriteRegister(_address, USER_CTRL, I2C_MST_EN | I2C_IF_DIS | I2C_MST_RST);
    } else {
        WriteRegister(_address, USER_CTRL, I2C_MST_EN | I2C_MST_RST);
    }

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
    // The 'downwards' gravitational pull on the accel probe mass causes the accel to output an
    // acceleration upwards. In NED coordinates the z-axis is positive down, therefore the upwards
    // acceleration (due to the gravitational pull) is negative. By convention +1G is a positive
    // vector that points downwards. The measured acceleration (due to gravity) has to be inverted.
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
    c = c | 0x01;           // Set gyro rate to 1kHz and bandwidth to 184Hz
    WriteRegister(_address, CONFIG, c);

    // Set accelerometer sampling rate and digital lp-filter config
    // TODO: Allow the user to specify rate + bw
    c = ReadRegister(_address, ACCEL_CONFIG2);
    c = c & ~0x0F; // Clear ACCEL_FCHOICE_B (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x01;  // Set accel rate to 1 kHz and bandwidth to 184Hz
    WriteRegister(_address, ACCEL_CONFIG2, c);

    // Set sensor data output rate = sampling rate/(1 + SMPLRT_DIV)
    WriteRegister(_address, SMPLRT_DIV, SRD);

    return 0;
}

void mpu9250::EnableInterrupt(int intPin, void(*irsFunc)())
{
    if (_useSPI) {
        // If SPI transactions are to be called from an interrupt the corresponding intPin
        // has to be registered
        _spiBus->usingInterrupt(intPin);

        // Config MPU9250 - SPI mode interrupt: Auto-clear on reg read
        WriteRegister(_address, INT_PIN_CFG, INT_ANYRD_2CLEAR);
    } else {
        // Config MPU9250 - I2C mode interrupt: Manual-clear

        // NOTE: In I2C master mode and INT_ANYRD_2CLEAR there is an issue with clears triggering
        // randomly before slave 0 register readout. This causes the I2C bus to stall. Use LATCH
        // mode and clear the interrupt manually to circumvent. This sadly adds about 125us to IRS.
        WriteRegister(_address, INT_PIN_CFG, INT_LATCH_EN);
    }
    // Enable/Attach uC interrupt pin and irs function
    pinMode(intPin, INPUT);
    attachInterrupt(intPin, irsFunc, RISING);

    // Enable MPU 9250 interrupt on new raw data
    WriteRegister(_address, INT_ENABLE, INT_RAW_RDY_EN);
    delay(100);
}

uint8_t mpu9250::ClearInterrupt()
{
    // If !INT_ANYRD_2CLEAR a 'manual' read of INT_STATUS clears the interrupt
    return ReadRegister(_address, INT_STATUS);
}

uint8_t mpu9250::EnableDMA()
{
    if (_useSPI) {
        // TODO: non-blocking SPI not implemented
        return 0;
    } else {
        return _i2cBus->setOpMode(I2C_OP_MODE_DMA);
    }
}

void mpu9250::GyroCal()
{
    // Configure device for bias calculation
    WriteRegister(_address, INT_ENABLE, 0x00);          // Disable all interrupts
    WriteRegister(_address, FIFO_EN, 0x00);             // Disable FIFO
    WriteRegister(_address, PWR_MGMT_1, CLKSEL_AUTO);   // Turn on internal clock source
    WriteRegister(_address, PWR_MGMT_2, SENS_EN);       // enable gyro (and accel)
    WriteRegister(_address, I2C_MST_CTRL, 0x00);        // Disable I2C master
    WriteRegister(_address, USER_CTRL, 0x00);           // Disable FIFO and I2C master modes
    WriteRegister(_address, USER_CTRL, FIFO_DMP_RST);   // Reset FIFO and DMP
    delay(15);

    // Configure MPU9250 gyro for bias calculation
    WriteRegister(_address, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
    WriteRegister(_address, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    WriteRegister(_address, GYRO_CONFIG, 0x00);  // Set gyro FS to 250 deg/s, maximum sensitivity

    // Wait a few for device to settle
    delay(200);

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    WriteRegister(_address, USER_CTRL, 0x40);   // Enable FIFO
    WriteRegister(_address, FIFO_EN, 0x70);     // Write gyro data to FIFO (max size 512 bytes)

    // Record for 60 ms -> 3x 2bytes/ms ~ 360 bytes @ 60ms
    // TODO: Use FIFO interrupt to count exact number of captured samples
    delay(60);

    // Stop and turn off FIFO
    WriteRegister(_address, FIFO_EN, 0x00);             // Disable gyro write to FIFO

    // Read FIFO sample count and get packet count
    uint8_t data[6];
    uint16_t packet_count, fifo_count;
    ReadRegisters(_address, FIFO_COUNTH, 2, &data[0]);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / sizeof(data);           // 3x 2bytes per gyro_axis

    // Sum data for averaging
    int32_t gyro_bias[3]  = { 0 };
    for (unsigned int i = 0; i < packet_count; i++) {
        int16_t gyro_temp[3] = { 0 };

        ReadRegisters(_address, FIFO_R_W, sizeof(data), &data[0]);
        gyro_temp[0]  = ((int16_t)data[0] << 8) | data[1];
        gyro_temp[1]  = ((int16_t)data[2] << 8) | data[3];
        gyro_temp[2]  = ((int16_t)data[4] << 8) | data[5];

        gyro_bias[0]  += (int32_t)gyro_temp[0];
        gyro_bias[1]  += (int32_t)gyro_temp[1];
        gyro_bias[2]  += (int32_t)gyro_temp[2];
    }

    // Get the average (biases)
    gyro_bias[0]  /= (int32_t)packet_count;
    gyro_bias[1]  /= (int32_t)packet_count;
    gyro_bias[2]  /= (int32_t)packet_count;

    // Split gyro biases into 8_BIT_H and 8_BIT_L, use conversion formula as per datasheet
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

#if 0
    float gyroBias[3];

    // Output scaled gyro biases for display in the main program
    gyroBias[0] = (float)gyro_bias[0] / GFSF_250DPS *_d2r;    // 131 LSB/degrees/sec (to rad)
    gyroBias[1] = (float)gyro_bias[1] / GFSF_250DPS *_d2r;
    gyroBias[2] = (float)gyro_bias[2] / GFSF_250DPS *_d2r;

    Serial.printf("gyro biases (rad)\n%f\n%f\n%f\n", gyroBias[0], gyroBias[1], gyroBias[2]);
#endif

    return;
}

void mpu9250::SetMagCal(float* magBias_in, float* magScale_in)
{
    _magHardIron[0] = magBias_in[0];
    _magHardIron[1] = magBias_in[1];
    _magHardIron[2] = magBias_in[2];
    _magSoftIron[0] = magScale_in[0];
    _magSoftIron[1] = magScale_in[1];
    _magSoftIron[2] = magScale_in[2];

#if 0
    Serial.printf("Preset calibration:\n");
    Serial.printf("AK8963 mag biases (mG)\n%f\n%f\n%f\n", _magHardIron[0], _magHardIron[1], _magHardIron[2]);
    Serial.printf("AK8963 mag scale (mG)\n%f\n%f\n%f\n\n", _magSoftIron[0], _magSoftIron[1], _magSoftIron[2]);

    Serial.printf("Factory calibration:\n");
    Serial.printf("Sensitivity scale (counts)\n%.2f\n%.2f\n%.2f\n\n",
                    _magScale_factory[0], _magScale_factory[1], _magScale_factory[2]);
#endif
}

void mpu9250::MagCal()
{
    int sample_count = 0;
    int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
    int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 };
    int16_t mag_counts[3] = { 0, 0, 0 };

    // TODO: Notify the user in some way that mag cal is about to start
#if 0
    Serial.printf("AK8963 initialized for active data mode...\n");
    Serial.printf("Mag Calibration: Wave device in a figure eight until done!\n");
    delay(4000);
#endif

    // shoot for ~thirty seconds of mag data
    if (_magRate == MAG_RATE_8HZ) sample_count = 30 * 8;      // at 8 Hz ODR, new mag data every 125 ms
    if (_magRate == MAG_RATE_100HZ) sample_count = 30 * 100;  // at 100 Hz ODR, new mag data every 10 ms

    for (int i = 0; i < sample_count; i++) {

        // Read mag counts from mpu9250 EXT_SENS_DATA_00 register
        uint8_t rawData[8];
        ReadRegisters(_address, EXT_SENS_DATA_00, sizeof(rawData), &rawData[0]);
       if ((rawData[0] & 0x01) && !(rawData[7] & 0x08)) {
            // Mag data is ready && no mag overflow
           mag_counts[0] = ((int16_t)rawData[2] << 8) | rawData[1];
           mag_counts[1] = ((int16_t)rawData[4] << 8) | rawData[3];
           mag_counts[2] = ((int16_t)rawData[6] << 8) | rawData[5];
        }

        for (int j = 0; j < 3; j++) {
            if (mag_counts[j] > mag_max[j]) mag_max[j] = mag_counts[j];
            if (mag_counts[j] < mag_min[j]) mag_min[j] = mag_counts[j];
        }

    if (_magRate == MAG_RATE_8HZ) delay(135);   // at 8 Hz ODR, new mag data every 125 ms
    if (_magRate == MAG_RATE_100HZ) delay(12);  // at 100 Hz ODR, new mag data every 10 ms
    }

#if 0
    Serial.printf("mag x min/max:%d / %d\n", mag_max[0], mag_min[0]);
    Serial.printf("mag y min/max:%d / %d\n", mag_max[1], mag_min[1]);
    Serial.printf("mag z min/max:%d / %d\n", mag_max[2], mag_min[2]);
#endif

    // Get hard iron correction for xyz axes in counts
    // This is the offset from the sphere/elipse to the origin
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

    // save mag biases in G for main program
    _magHardIron[0] = (float)mag_bias[0]; // * _magScale * _magScale_factory[0];
    _magHardIron[1] = (float)mag_bias[1]; // * _magScale * _magScale_factory[1];
    _magHardIron[2] = (float)mag_bias[2]; // * _magScale * _magScale_factory[2];

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
    Serial.printf("Mag calibration done!\n");
    Serial.printf("AK8963 mag biases (counts)\n%f\n%f\n%f\n", _magHardIron[0], _magHardIron[1], _magHardIron[2]);
    Serial.printf("AK8963 mag scale  \n%f\n%f\n%f\n\n", _magSoftIron[0], _magSoftIron[1], _magSoftIron[2]);

    Serial.printf("Factory calibration:\n");
    Serial.printf("Sensitivity scale \n%.2f\n%.2f\n%.2f\n\n",
                    _magScale_factory[0], _magScale_factory[1], _magScale_factory[2]);
#endif
}

void mpu9250::SelfTest()
{
    uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
    uint8_t selfTest[6];
    int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] = { 0 };

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

#if 0
    float factoryTrim[6], deviation[6];

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620 / 1 << ACCEL_RANGE_2G) * (pow(1.01, ((float)selfTest[0] - 1.0)));
    factoryTrim[1] = (float)(2620 / 1 << ACCEL_RANGE_2G) * (pow(1.01, ((float)selfTest[1] - 1.0)));
    factoryTrim[2] = (float)(2620 / 1 << ACCEL_RANGE_2G) * (pow(1.01, ((float)selfTest[2] - 1.0)));
    factoryTrim[3] = (float)(2620 / 1 << GYRO_RANGE_250DPS) * (pow(1.01, ((float)selfTest[3] - 1.0)));
    factoryTrim[4] = (float)(2620 / 1 << GYRO_RANGE_250DPS) * (pow(1.01, ((float)selfTest[4] - 1.0)));
    factoryTrim[5] = (float)(2620 / 1 << GYRO_RANGE_250DPS) * (pow(1.01, ((float)selfTest[5] - 1.0)));

    // Report results as a ratio of (STR - FT)/FT in percent
    for (int i = 0; i < 3; i++) {
        deviation[i]   = 100.0f * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.0f;
        deviation[i+3] = 100.0f * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i+3] - 100.0f;
    }

    // Print the values
    Serial.printf("x-axis self test: accel trim within: %.1f \% of factory value\n", deviation[0]);
    Serial.printf("y-axis self test: accel trim within: %.1f \% of factory value\n", deviation[1]);
    Serial.printf("z-axis self test: accel trim within: %.1f \% of factory value\n", deviation[2]);

    Serial.printf("x-axis self test: gyro trim within: %.1f \% of factory value\n", deviation[3]);
    Serial.printf("y-axis self test: gyro trim within: %.1f \% of factory value\n", deviation[4]);
    Serial.printf("z-axis self test: gyro trim within: %.1f \% of factory value\n\n", deviation[5]);
#endif
}

bool mpu9250::WriteRegister(uint8_t address, uint8_t subAddress, uint8_t data_in)
{
    uint8_t buff[1];

    if (_useSPI) {
        _spiBus->beginTransaction(SPISettings(SPILS_CLK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin, LOW);      // select the MPU9250 chip
        _spiBus->transfer(subAddress);      // write the register address
        _spiBus->transfer(data_in);         // write the data
        digitalWriteFast(_csPin, HIGH);     // deselect the MPU9250 chip
        _spiBus->endTransaction();          // end the transaction

    } else {
        _i2cBus->beginTransmission(address);  // Initialize the Tx buffer
        _i2cBus->write(subAddress);           // Put slave register address in Tx buffer
        _i2cBus->write(data_in);              // Put data in Tx buffer
        _i2cBus->endTransmission();           // Send the Tx buffer
    }
    delay(10); // need to slow down how fast we write to MPU9250

    // read back the register
    ReadRegisters(_address, subAddress, sizeof(buff), &buff[0]);
    return (buff[0] == data_in);
}

void mpu9250::SendRegister(uint8_t address, uint8_t subAddress, uint8_t data_in)
{
    if (_useSPI) {
        // TODO: non-blocking SPI not implemented
    } else {
        _i2cBus->beginTransmission(address);  // Initialize the Tx buffer
        _i2cBus->write(subAddress);           // Put slave register address in Tx buffer
        _i2cBus->write(data_in);              // Put data in Tx buffer
        _i2cBus->sendTransmission();          // Send the Tx buffer
    }
}

void mpu9250::ReadRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* data_out)
{
    if (_useSPI) {
        _spiBus->beginTransaction(SPISettings(_useSPIHS ? SPIHS_CLK : SPILS_CLK, MSBFIRST, SPI_MODE3));

        // WORKAROUND: Trigger a pre-transfer to avoid a bug where SPI would
        // report data being 1 frame old

        digitalWriteFast(_csPin, LOW);              // select the MPU9250 chip
        _spiBus->transfer(subAddress | SPI_READ);   // specify the starting register address
        digitalWriteFast(_csPin, HIGH);             // deselect the MPU9250 chip
        delayMicroseconds(1);

        // Proceed with the actual transaction
        digitalWriteFast(_csPin, LOW);              // select the MPU9250 chip
        _spiBus->transfer(subAddress | SPI_READ);   // specify the starting register address

        for (uint8_t i = 0; i < count; i++)
            data_out[i] = _spiBus->transfer(0x00);  // read the data

        digitalWriteFast(_csPin, HIGH);             // deselect the MPU9250 chip
        _spiBus->endTransaction();                  // end the transaction
    } else {

        _i2cBus->beginTransmission(address);                      // Begin i2c with slave at address
        _i2cBus->write(subAddress);                               // Put slave register addr in tx buffer
        _i2cBus->endTransmission(I2C_NOSTOP);                     // Send the tx buffer, keep connection alive
        _i2cBus->requestFrom(address, (size_t) count, I2C_STOP);  // Blocking request of 'count' data at subaddr

        // Get the received data
        uint8_t i = 0;
        while (_i2cBus->available())
            data_out[i++] = _i2cBus->readByte();
    }
}

uint8_t mpu9250::ReadRegister(uint8_t address, uint8_t subAddress)
{
    uint8_t data;
    ReadRegisters(address, subAddress, 1, &data);
    return data;
}

void mpu9250::RequestRegisters(uint8_t address, uint8_t subAddress, uint8_t count)
{
    if (_useSPI) {
        // TODO: non-blocking SPI not implemented
    } else {
        _i2cBus->beginTransmission(address);                      // Begin i2c with slave at address
        _i2cBus->write(subAddress);                               // Put slave register addr in tx buffer
        _i2cBus->endTransmission(I2C_NOSTOP);                     // Send the tx buffer, keep connection alive
        _i2cBus->sendRequest(address, (size_t) count, I2C_STOP);  // Non-blocking request of 'count' data at subaddr
    }
}

void mpu9250::ReadRequested(uint8_t* data_out)
{
    if (_useSPI) {
        // TODO: non-blocking SPI not implemented
    } else {
        // Get the received data
        uint8_t i = 0;
        while (_i2cBus->available())
            data_out[i++] = _i2cBus->readByte();
    }
}

uint8_t mpu9250::RequestedAvailable()
{
    if (_useSPI) {
        // TODO: non-blocking SPI not implemented
    }

    if (!_useSPI && _requestedData && _i2cBus->done()) {
        return !(_requestedData = false);
    }

    return false;
}

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
    return (buff[0] == data);
}

void mpu9250::ReadAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* data_out)
{
    // Config SLV0 for normal operation
    WriteRegister(_address, I2C_SLV0_ADDR, I2C_READ_FLAG | AK8963_ADDRESS); // SLV0 to AK8963 & set for read
    WriteRegister(_address, I2C_SLV0_REG, subAddress);                      // AK8963 register to read from
    WriteRegister(_address, I2C_SLV0_CTRL, I2C_SLV0_EN | count);            // Enable I2C to send 'count' bytes

    delayMicroseconds(100);     // takes some time for these registers to fill

    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    ReadRegisters(_address, EXT_SENS_DATA_00, count, data_out);
}

uint8_t mpu9250::whoAmI()
{
    uint8_t buff[1];

    // read the WHO AM I register
    ReadRegisters(_address, WHO_AM_I_MPU9250, sizeof(buff), &buff[0]);
    return buff[0];
}

uint8_t mpu9250::whoAmIAK8963()
{
    uint8_t buff[1];

    // read the WHO AM I register
    ReadAK8963Registers(AK8963_WHO_AM_I, sizeof(buff), &buff[0]);
    return buff[0];
}
