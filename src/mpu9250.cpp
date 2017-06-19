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

void mpu9250::SetMres(uint8_t scale)
{
    switch (scale) {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
        _mRes = 10.*4912./8190.; // Proper scale to return milliGauss
        break;
    case MFS_16BITS:
        _mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
        break;
    }
    _Mscale = scale;
}

void mpu9250::SetGres(uint8_t scale)
{
    switch (scale) {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
        _gRes = 250.0/32767.5;
        break;
    case GFS_500DPS:
        _gRes = 500.0/32767.5;
        break;
    case GFS_1000DPS:
        _gRes = 1000.0/32767.5;
        break;
    case GFS_2000DPS:
        _gRes = 2000.0/32767.5;
        break;
    }
    _Gscale = scale;
}

void mpu9250::SetAres(uint8_t scale)
{
    switch (scale) {
    // data registers hold 2 bytes -> int16_t
    // With int16_t = 2^16 bits our range is [-32768, 32767] (1 sign bit)
    // resolution per bit = FS / 32767.5f
    case AFS_2G:
        _aRes = 2.0/32767.5f;
        break;
    case AFS_4G:
        _aRes = 4.0/32767.5;
        break;
    case AFS_8G:
        _aRes = 8.0/32767.5f;
        break;
    case AFS_16G:
        _aRes = 16.0/32767.5f;
        break;
    }
    _Ascale = scale;
}

void mpu9250::SetMmode(uint8_t mode)
{
    _Mmode = mode;
}


void mpu9250::ReadMPU9250Data(int16_t * destination)
{
    uint8_t rawData[14];  // x/y/z accel register data stored here
    ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
    destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
    destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
    destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
    destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void mpu9250::ReadAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void mpu9250::ReadGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

bool mpu9250::NewMagData()
{
    if(_newMagData) {
        _newMagData = false;
        return true;
    }
    return false;
}
void mpu9250::ReadMagData(int16_t * destination)
{
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    _newMagData = (ReadByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);

    if(_newMagData == true) { // wait for magnetometer data ready bit to be set
        ReadBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register

        if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
            destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
        }
    }
}

int16_t mpu9250::ReadTempData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    ReadBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void mpu9250::InitAK8963(float * destination)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z mag calibration data stored here
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    delay(10);
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);
    // Read the x-, y-, and z-axis factory calibration values
    ReadBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);
    _magCalFactory[0] = destination[0] =  (float)(rawData[0] - 128)/256. + 1.;
    _magCalFactory[1] = destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
    _magCalFactory[2] = destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    delay(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, _Mscale << 4 | _Mmode); // Set magnetometer data resolution and sample ODR
    delay(10);
}


void mpu9250::Init()
{
    // wake up device
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    delay(100); // Wait for all registers to reset

    // get stable time source
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    delay(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    WriteByte(MPU9250_ADDRESS, CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = ReadByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x03; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear GFS bits [4:3]
    c = c | _Gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | _Ascale << 3; // Set full scale range for the accelerometer
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    // WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear
    WriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    delay(100);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void mpu9250::AcelGyroCal(float * dest1, float * dest2)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    WriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    WriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    WriteByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
    WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    ReadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        ReadBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

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
    WriteByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    WriteByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    WriteByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    WriteByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    WriteByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    WriteByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    ReadBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    ReadBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    ReadBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1UL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
        if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
#if 0
    WriteByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
    WriteByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
    WriteByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
    WriteByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
    WriteByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
    WriteByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
#endif

    // Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void mpu9250::SetMagCal(float *magBias, float *magScale)
{
    _magBias[0] = magBias[0];
    _magBias[1] = magBias[1];
    _magBias[2] = magBias[2];
    _magScale[0] = magScale[0];
    _magScale[1] = magScale[1];
    _magScale[2] = magScale[2];
}

void mpu9250::MagCal(float * dest1, float * dest2)
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    // shoot for ~thirty seconds of mag data
    if(_Mmode == MRATE_8HZ) sample_count = 30*8;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(_Mmode == MRATE_100HZ) sample_count = 30*100;  // at 100 Hz ODR, new mag data is available every 10 ms

    for(ii = 0; ii < sample_count; ii++) {
        ReadMagData(mag_temp);  // Read the mag data

        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }

    if(_Mmode == MRATE_8HZ) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(_Mmode == MRATE_100HZ) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts


    dest1[0] = (float) mag_bias[0]*_mRes * _magCalFactory[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*_mRes * _magCalFactory[1];
    dest1[2] = (float) mag_bias[2]*_mRes * _magCalFactory[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
}

void mpu9250::SelfTest(float * destination)
{
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float factoryTrim[6];
    uint8_t FS = 0; // Max sensitivity AFS_2G and GFS_250DPS

    WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);       // Gyro sample rate to 1 kHz
    WriteByte(MPU9250_ADDRESS, CONFIG, 0x02);           // Gyro sample rate to 1 kHz and DLPF to 92 Hz
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3);   // Gyro scale to 250 dps
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);    // Accel rate to 1 kHz and bandwidth to 92 Hz
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3);  // Accel scale to 2g

    // average 200 gyro and acclerometer values
    for( int ii = 0; ii < 200; ii++) {

        // Read raw accel data for 8bit reg pairs and convert to int16_t
        ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        // Read raw gyro data for 8bit reg pairs and convert to int16_t
        ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
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
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0);
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0);
    delay(25);  // Delay a while to let the device stabilize

    for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

        // Read raw accel data for 8bit reg pairs and convert to int16_t
        ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        // Read raw gyro data for 8bit reg pairs and convert to int16_t
        ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
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
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
    delay(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test result from USR_Reg
    selfTest[0] = ReadByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL);
    selfTest[1] = ReadByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL);
    selfTest[2] = ReadByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL);
    selfTest[3] = ReadByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);
    selfTest[4] = ReadByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);
    selfTest[5] = ReadByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) ));
    factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) ));
    factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) ));
    factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) ));
    factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) ));
    factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) ));

    // Report results as a ratio of (STR - FT)/FT in percent
    for (int i = 0; i < 3; i++) {
        destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;
        destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.;
    }
    // Return percent deviation from factory trim values, +/- 14 or less deviation is a pass
}

// I2C read/write functions for the MPU9250 and AK8963 sensors
void mpu9250::WriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t mpu9250::ReadByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data;                            // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.requestFrom(address, 1);            // Read one byte from slave register address
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void mpu9250::ReadBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
//        Wire.requestFrom(address, count);     // Read bytes from slave register address
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
    while (Wire.available()) {
        dest[i++] = Wire.read();       // Put read results in the Rx buffer
    }
}
