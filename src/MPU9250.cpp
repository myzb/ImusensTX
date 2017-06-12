/*
MPU9250.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2017-01-04

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

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC 
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
	defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "MPU9250.h"
#include "i2c_t3.h"  // I2C library
#include "SPI.h" // SPI Library

/* MPU9250 object, input the I2C address and I2C bus */
MPU9250::MPU9250(uint8_t address, uint8_t bus){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _userDefI2C = false; // automatic I2C setup
    _useSPI = false; // set to use I2C instead of SPI
}

/* MPU9250 object, input the I2C address, I2C bus, and I2C pins */
MPU9250::MPU9250(uint8_t address, uint8_t bus, i2c_pins pins){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _pins = pins; // I2C pins
    _pullups = I2C_PULLUP_EXT; // I2C pullups
    _userDefI2C = true; // user defined I2C
    _useSPI = false; // set to use I2C instead of SPI
}

/* MPU9250 object, input the I2C address, I2C bus, I2C pins, and I2C pullups */
MPU9250::MPU9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _pins = pins; // I2C pins
    _pullups = pullups; // I2C pullups
    _userDefI2C = true; // user defined I2C
    _useSPI = false; // set to use I2C instead of SPI
}

/* MPU9250 object, input the SPI CS Pin */
MPU9250::MPU9250(uint8_t csPin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = MOSI_PIN_11;	// SPI MOSI Pin, set to default
    _useSPI = true; // set to use SPI instead of I2C
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

/* MPU9250 object, input the SPI CS Pin and MOSI Pin */
MPU9250::MPU9250(uint8_t csPin, spi_mosi_pin pin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = pin;	// SPI MOSI Pin
    _useSPI = true; // set to use SPI instead of I2C
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

/* starts I2C communication and sets up the MPU-9250 */
int MPU9250::begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange){
    uint8_t buff[3];
    uint8_t data[7];

    if( _useSPI ){ // using SPI for communication

        // setting CS pin to output
        pinMode(_csPin,OUTPUT);

        // setting CS pin high
        digitalWriteFast(_csPin,HIGH);

        // Teensy 3.0 || Teensy 3.1/3.2
		#if defined(__MK20DX128__) || defined(__MK20DX256__)

	        // configure and begin the SPI
	        switch( _mosiPin ){

				case MOSI_PIN_7:	// SPI bus 0 alternate 1
				    SPI.setMOSI(7);
	        		SPI.setMISO(8);
	        		SPI.setSCK(14);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_11:	// SPI bus 0 default
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

	        	case MOSI_PIN_0:	// SPI bus 1 default
	        		SPI1.setMOSI(0);
	        		SPI1.setMISO(1);
	        		SPI1.setSCK(32);
	        		SPI1.begin();
	        		break;
				case MOSI_PIN_7:	// SPI bus 0 alternate 1
				    SPI.setMOSI(7);
	        		SPI.setMISO(8);
	        		SPI.setSCK(14);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_11:	// SPI bus 0 default
					SPI.setMOSI(11);
	        		SPI.setMISO(12);
	        		SPI.setSCK(13);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_21:	// SPI bus 1 alternate
		        	SPI1.setMOSI(21);
	        		SPI1.setMISO(5);
	        		SPI1.setSCK(20);
	        		SPI1.begin();
	        		break;
				case MOSI_PIN_28:	// SPI bus 0 alternate 2
	        		SPI.setMOSI(28);
	        		SPI.setMISO(39);
	        		SPI.setSCK(27);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_44:	// SPI bus 2 default
	        		SPI2.setMOSI(44);
	        		SPI2.setMISO(45);
	        		SPI2.setSCK(46);
	        		SPI2.begin();
	        		break;
				case MOSI_PIN_52:	// SPI bus 2 alternate
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

	        	case MOSI_PIN_0:	// SPI bus 1 default
	        		SPI1.setMOSI(0);
	        		SPI1.setMISO(1);
	        		SPI1.setSCK(20);
	        		SPI1.begin();
	        		break;
				case MOSI_PIN_7:	// SPI bus 0 alternate 1
				    SPI.setMOSI(7);
	        		SPI.setMISO(8);
	        		SPI.setSCK(14);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_11:	// SPI bus 0 default
					SPI.setMOSI(11);
	        		SPI.setMISO(12);
	        		SPI.setSCK(13);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_21:	// SPI bus 1 alternate
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
        i2c_t3(_bus).begin(I2C_MASTER, 0, _pins, _pullups, _i2cRate);
    }

    // select clock source to gyro
    if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
        return -1;
    }

    // enable I2C master mode
    if( !writeRegister(USER_CTRL,I2C_MST_EN) ){
        return -1;
    }

    // set the I2C bus speed to 400 kHz
    if( !writeRegister(I2C_MST_CTRL,I2C_MST_CLK) ){
        return -1;
    }

    // set AK8963 to Power Down
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) ){
        return -1;
    }

    // reset the MPU9250
    writeRegister(PWR_MGMNT_1,PWR_RESET);

    // wait for MPU-9250 to come back up
    delay(1);

    // reset the AK8963
    writeAK8963Register(AK8963_CNTL2,AK8963_RESET);

    // select clock source to gyro
    if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
        return -1;
    }

    // check the WHO AM I byte, expected value is 0x71 (decimal 113)
    if( whoAmI() != 113 ){
        return -1;
    }

    // enable accelerometer and gyro
    if( !writeRegister(PWR_MGMNT_2,SEN_ENABLE) ){
        return -1;
    }

    /* setup the accel and gyro ranges */
    switch(accelRange) {

        case ACCEL_RANGE_2G:
            // setting the accel range to 2G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) ){
                return -1;
            }
            _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
            break;

        case ACCEL_RANGE_4G:
            // setting the accel range to 4G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) ){
                return -1;
            }
            _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
            break;

        case ACCEL_RANGE_8G:
            // setting the accel range to 8G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) ){
                return -1;
            }
            _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
            break;

        case ACCEL_RANGE_16G:
            // setting the accel range to 16G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) ){
                return -1;
            }
            _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
            break;
    }

    switch(gyroRange) {
        case GYRO_RANGE_250DPS:
            // setting the gyro range to 250DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) ){
                return -1;
            }
            _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
            break;

        case GYRO_RANGE_500DPS:
            // setting the gyro range to 500DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) ){
                return -1;
            }
            _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
            break;

        case GYRO_RANGE_1000DPS:
            // setting the gyro range to 1000DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) ){
                return -1;
            }
            _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
            break;

        case GYRO_RANGE_2000DPS:
            // setting the gyro range to 2000DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) ){
                return -1;
            }
            _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
            break;
    }

    // enable I2C master mode
    if( !writeRegister(USER_CTRL,I2C_MST_EN) ){
    	return -1;
    }

	// set the I2C bus speed to 400 kHz
	if( !writeRegister(I2C_MST_CTRL,I2C_MST_CLK) ){
		return -1;
	}

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 72 ){
        return -1;
	}

    /* get the magnetometer calibration */

    // set AK8963 to Power Down
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) ){
        return -1;
    }
    delay(100); // long wait between AK8963 mode changes

    // set AK8963 to FUSE ROM access
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) ){
        return -1;
    }
    delay(100); // long wait between AK8963 mode changes

    // read the AK8963 ASA registers and compute magnetometer scale factors
    readAK8963Registers(AK8963_ASA,sizeof(buff),&buff[0]);
    _magScaleX = ((((float)buff[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleY = ((((float)buff[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleZ = ((((float)buff[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla 

    // set AK8963 to Power Down
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) ){
        return -1;
    }
    delay(100); // long wait between AK8963 mode changes  

    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) ){
        return -1;
    }
    delay(100); // long wait between AK8963 mode changes

    // select clock source to gyro
    if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
        return -1;
    }       

    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,sizeof(data),&data[0]);

    // successful init, return 0
    return 0;
}


/* sets the DLPF and interrupt settings */
int MPU9250::setFilt(mpu9250_dlpf_bandwidth bandwidth, uint8_t SRD){
    uint8_t data[7];

    switch(bandwidth) {
        case DLPF_BANDWIDTH_184HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) ){ // setting accel bandwidth to 184Hz
                return -1;
            } 
            if( !writeRegister(CONFIG,GYRO_DLPF_184) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_92HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_92) ){ // setting accel bandwidth to 92Hz
                return -1;
            } 
            if( !writeRegister(CONFIG,GYRO_DLPF_92) ){ // setting gyro bandwidth to 92Hz
                return -1;
            }
            break; 

        case DLPF_BANDWIDTH_41HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_41) ){ // setting accel bandwidth to 41Hz
                return -1;
            } 
            if( !writeRegister(CONFIG,GYRO_DLPF_41) ){ // setting gyro bandwidth to 41Hz
                return -1;
            } 
            break;

        case DLPF_BANDWIDTH_20HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20) ){ // setting accel bandwidth to 20Hz
                return -1;
            } 
            if( !writeRegister(CONFIG,GYRO_DLPF_20) ){ // setting gyro bandwidth to 20Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_10HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10) ){ // setting accel bandwidth to 10Hz
                return -1;
            } 
            if( !writeRegister(CONFIG,GYRO_DLPF_10) ){ // setting gyro bandwidth to 10Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_5HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5) ){ // setting accel bandwidth to 5Hz
                return -1;
            } 
            if( !writeRegister(CONFIG,GYRO_DLPF_5) ){ // setting gyro bandwidth to 5Hz
                return -1;
            }
            break; 
    }

    /* setting the sample rate divider */
    if( !writeRegister(SMPDIV,SRD) ){ // setting the sample rate divider
        return -1;
    } 

    if(SRD > 9){

        // set AK8963 to Power Down
        if( !writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) ){
            return -1;
        }
        delay(100); // long wait between AK8963 mode changes  

        // set AK8963 to 16 bit resolution, 8 Hz update rate
        if( !writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) ){
            return -1;
        }
        delay(100); // long wait between AK8963 mode changes     

        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK8963_HXL,sizeof(data),&data[0]);
    }

    /* setting the interrupt */
    if( !writeRegister(INT_PIN_CFG,INT_PULSE_50US) ){ // setup interrupt, 50 us pulse
        return -1;
    }  
    if( !writeRegister(INT_ENABLE,INT_RAW_RDY_EN) ){ // set to data ready
        return -1;
    }  

    // successful filter setup, return 0
    return 0; 
}

/* enables and disables the interrupt */
int MPU9250::enableInt(bool enable){

	if(enable){
		/* setting the interrupt */
	    if( !writeRegister(INT_PIN_CFG,INT_PULSE_50US) ){ // setup interrupt, 50 us pulse
	        return -1;
	    }  
	    if( !writeRegister(INT_ENABLE,INT_RAW_RDY_EN) ){ // set to data ready
	        return -1;
	    }  
	}
	else{
	    if( !writeRegister(INT_ENABLE,INT_DISABLE) ){ // disable interrupt
	        return -1;
	    }  
	}

    // successful interrupt setup, return 0
    return 0; 
}


/* get accelerometer data given pointers to store the three values, return data as counts */
void MPU9250::getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az){
    uint8_t buff[6];
    int16_t axx, ayy, azz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz; // transform axes
    *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
    *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;
}

/* get accelerometer data given pointers to store the three values */
void MPU9250::getAccel(float* ax, float* ay, float* az){
    int16_t accel[3];

    getAccelCounts(&accel[0], &accel[1], &accel[2]);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;
}

/* get gyro data given pointers to store the three values, return data as counts */
void MPU9250::getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz){
    uint8_t buff[6];
    int16_t gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(GYRO_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    gxx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    gyy = (((int16_t)buff[2]) << 8) | buff[3];
    gzz = (((int16_t)buff[4]) << 8) | buff[5];

    *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz; // transform axes
    *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
    *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;
}

/* get gyro data given pointers to store the three values */
void MPU9250::getGyro(float* gx, float* gy, float* gz){
    int16_t gyro[3];

    getGyroCounts(&gyro[0], &gyro[1], &gyro[2]);

    *gx = ((float) gyro[0]) * _gyroScale; // typecast and scale to values
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;
}

/* get magnetometer data given pointers to store the three values, return data as counts */
void MPU9250::getMagCounts(int16_t* hx, int16_t* hy, int16_t* hz){
    uint8_t buff[7];
    _useSPIHS = true; // use the high speed SPI for data readout

    // read the magnetometer data off the external sensor buffer
    readRegisters(EXT_SENS_DATA_00,sizeof(buff),&buff[0]);

    if( buff[6] == 0x10 ) { // check for overflow
        *hx = (((int16_t)buff[1]) << 8) | buff[0];  // combine into 16 bit values
        *hy = (((int16_t)buff[3]) << 8) | buff[2];
        *hz = (((int16_t)buff[5]) << 8) | buff[4];
    }
    else{
        *hx = 0;  
        *hy = 0;
        *hz = 0;
    }
}

/* get magnetometer data given pointers to store the three values */
void MPU9250::getMag(float* hx, float* hy, float* hz){
    int16_t mag[3];

    getMagCounts(&mag[0], &mag[1], &mag[2]);

    *hx = ((float) mag[0]) * _magScaleX; // typecast and scale to values
    *hy = ((float) mag[1]) * _magScaleY;
    *hz = ((float) mag[2]) * _magScaleZ;
}

/* get temperature data given pointer to store the value, return data as counts */
void MPU9250::getTempCounts(int16_t* t){
    uint8_t buff[2];
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(TEMP_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    *t = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit value and return
}

/* get temperature data given pointer to store the values */
void MPU9250::getTemp(float* t){
    int16_t tempCount;

    getTempCounts(&tempCount);

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset;
}

/* get accelerometer and gyro data given pointers to store values, return data as counts */
void MPU9250::getMotion6Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz){
    uint8_t buff[14];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz; // transform axes
    *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
    *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;

    *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz;
    *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
    *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;
}

/* get accelerometer and gyro data given pointers to store values */
void MPU9250::getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz){
    int16_t accel[3];
    int16_t gyro[3];

    getMotion6Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2]);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;
}

/* get accelerometer, gyro and temperature data given pointers to store values, return data as counts */
void MPU9250::getMotion7Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t){
    uint8_t buff[14];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *t = (((int16_t)buff[6]) << 8) | buff[7];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz; // transform axes
    *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
    *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;

    *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz;
    *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
    *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;
}

/* get accelerometer, gyro, and temperature data given pointers to store values */
void MPU9250::getMotion7(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* t){
    int16_t accel[3];
    int16_t gyro[3];
    int16_t tempCount;

    getMotion7Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &tempCount);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset; 
}

/* get accelerometer, gyro and magnetometer data given pointers to store values, return data as counts */
void MPU9250::getMotion9Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz){
    uint8_t buff[21];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *hx = (((int16_t)buff[15]) << 8) | buff[14];  
    *hy = (((int16_t)buff[17]) << 8) | buff[16];
    *hz = (((int16_t)buff[19]) << 8) | buff[18];

    *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz; // transform axes
    *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
    *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;

    *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz;
    *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
    *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;
}

/* get accelerometer, gyro, and magnetometer data given pointers to store values */
void MPU9250::getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz){
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];

    getMotion9Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2]);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;

    *hx = ((float) mag[0]) * _magScaleX;
    *hy = ((float) mag[1]) * _magScaleY;
    *hz = ((float) mag[2]) * _magScaleZ;
}

/* get accelerometer, magnetometer, and temperature data given pointers to store values, return data as counts */
void MPU9250::getMotion10Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz, int16_t* t){
    uint8_t buff[21];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *t = (((int16_t)buff[6]) << 8) | buff[7];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *hx = (((int16_t)buff[15]) << 8) | buff[14];
    *hy = (((int16_t)buff[17]) << 8) | buff[16];
    *hz = (((int16_t)buff[19]) << 8) | buff[18];

    *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz; // transform axes
    *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
    *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;

    *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz;
    *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
    *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;
}

void MPU9250::getMotion10(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz, float* t){
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];
    int16_t tempCount;

    getMotion10Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2], &tempCount);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;

    *hx = ((float) mag[0]) * _magScaleX;
    *hy = ((float) mag[1]) * _magScaleY;
    *hz = ((float) mag[2]) * _magScaleZ;

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset; 
}

/* writes a byte to MPU9250 register given a register address and data */
bool MPU9250::writeRegister(uint8_t subAddress, uint8_t data){
    uint8_t buff[1];

    /* write data to device */
    if( _useSPI ){

    	// Teensy 3.0 || Teensy 3.1/3.2
		#if defined(__MK20DX128__) || defined(__MK20DX256__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
		        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI.transfer(subAddress); // write the register address
		        SPI.transfer(data); // write the data
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
		        SPI.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI1.transfer(subAddress); // write the register address
		        SPI1.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI1.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
		    	SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI2.transfer(subAddress); // write the register address
		        SPI2.transfer(data); // write the data
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
		        SPI.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI1.transfer(subAddress); // write the register address
		        SPI1.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI1.endTransaction(); // end the transaction
	    	}

    	#endif
    }
    else{
      	i2c_t3(_bus).beginTransmission(_address); // open the device
      	i2c_t3(_bus).write(subAddress); // write the register address
      	i2c_t3(_bus).write(data); // write the data
      	i2c_t3(_bus).endTransmission();
    }
    delay(10); // need to slow down how fast I write to MPU9250

  	/* read back the register */
  	readRegisters(subAddress,sizeof(buff),&buff[0]);

  	/* check the read back register against the written register */
  	if(buff[0] == data) {
  		return true;
  	}
  	else{
  		return false;
  	}
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void MPU9250::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){

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
		            dest[i] = SPI.transfer(0x00); // read the data
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
		            dest[i] = SPI.transfer(0x00); // read the data
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
		            dest[i] = SPI1.transfer(0x00); // read the data
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
		            dest[i] = SPI.transfer(0x00); // read the data
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
		            dest[i] = SPI.transfer(0x00); // read the data
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
		            dest[i] = SPI1.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI1.endTransaction(); // end the transaction
	    	}

    	#endif
    }
    else{
        i2c_t3(_bus).beginTransmission(_address); // open the device
        i2c_t3(_bus).write(subAddress); // specify the starting register address
        i2c_t3(_bus).endTransmission(false);

        i2c_t3(_bus).requestFrom(_address, count); // specify the number of bytes to receive

        uint8_t i = 0; // read the data into the buffer
        while( i2c_t3(_bus).available() ){
            dest[i++] = i2c_t3(_bus).readByte();
        }
    }
}

/* writes a register to the AK8963 given a register address and data */
bool MPU9250::writeAK8963Register(uint8_t subAddress, uint8_t data){
	uint8_t count = 1;
	uint8_t buff[1];

	writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR); // set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_REG,subAddress); // set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_DO,data); // store the data for write
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count); // enable I2C and send 1 byte

	// read the register and confirm
	readAK8963Registers(subAddress, sizeof(buff), &buff[0]);

	if(buff[0] == data) {
  		return true;
  	}
  	else{
  		return false;
  	}
}

/* reads registers from the AK8963 */
void MPU9250::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){

	writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG); // set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_REG,subAddress); // set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count); // enable I2C and request the bytes
	delayMicroseconds(100); // takes some time for these registers to fill
	readRegisters(EXT_SENS_DATA_00,count,dest); // read the bytes off the MPU9250 EXT_SENS_DATA registers
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
uint8_t MPU9250::whoAmI(){
    uint8_t buff[1];

    // read the WHO AM I register
    readRegisters(WHO_AM_I,sizeof(buff),&buff[0]);

    // return the register value
    return buff[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
uint8_t MPU9250::whoAmIAK8963(){
    uint8_t buff[1];

    // read the WHO AM I register
    readAK8963Registers(AK8963_WHO_AM_I,sizeof(buff),&buff[0]);

    // return the register value
    return buff[0];
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250::accelGyroCal()
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  writeRegister(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  writeRegister(PWR_MGMT_1, 0x01);  
  writeRegister(PWR_MGMT_2, 0x00);
  delay(200);                                    

// Configure device for bias calculation
  writeRegister(INT_ENABLE, 0x00);   // Disable all interrupts
  writeRegister(FIFO_EN, 0x00);      // Disable FIFO
  writeRegister(PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeRegister(I2C_MST_CTRL, 0x00); // Disable I2C master
  writeRegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeRegister(USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeRegister(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeRegister(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeRegister(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeRegister(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeRegister(USER_CTRL, 0x40);   // Enable FIFO  
  writeRegister(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeRegister(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readRegisters(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readRegisters(FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
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
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  writeRegister(XG_OFFSET_H, data[0]);
  writeRegister(XG_OFFSET_L, data[1]);
  writeRegister(YG_OFFSET_H, data[2]);
  writeRegister(YG_OFFSET_L, data[3]);
  writeRegister(ZG_OFFSET_H, data[4]);
  writeRegister(ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  _gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  _gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  _gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readRegisters(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readRegisters(YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readRegisters(ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
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
/*  
  writeRegister(XA_OFFSET_H, data[0]);
  writeRegister(XA_OFFSET_L, data[1]);
  writeRegister(YA_OFFSET_H, data[2]);
  writeRegister(YA_OFFSET_L, data[3]);
  writeRegister(ZA_OFFSET_H, data[4]);
  writeRegister(ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for display in the main program
   _accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   _accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
   _accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::selfTest() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeRegister(SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeRegister(CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeRegister(GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  writeRegister(ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeRegister(ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  readRegisters(ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
  readRegisters(GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeRegister( ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeRegister( GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  readRegisters(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
  readRegisters(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   writeRegister(ACCEL_CONFIG, 0x00);  
   writeRegister(GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   readRegisters(SELF_TEST_X_ACCEL, 1, &selfTest[0]); // X-axis accel self-test results
   readRegisters(SELF_TEST_Y_ACCEL, 1, &selfTest[1]); // Y-axis accel self-test results
   readRegisters(SELF_TEST_Z_ACCEL, 1, &selfTest[2]); // Z-axis accel self-test results
   readRegisters(SELF_TEST_X_GYRO,  1, &selfTest[3]); // X-axis gyro self-test results
   readRegisters(SELF_TEST_Y_GYRO,  1, &selfTest[4]); // Y-axis gyro self-test results
   readRegisters(SELF_TEST_Z_GYRO,  1, &selfTest[5]); // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     _selfTest[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     _selfTest[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }
}

void MPU9250::magCal() 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  float mRes = 10.*4912./32760.0; // Proper scale to return milliGauss

  //Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
    // shoot for ~fifteen seconds of mag data
    //if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    //if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
   sample_count = 1500;
   
   for(ii = 0; ii < sample_count; ii++) {
    getMagCounts(&mag_temp[0], &mag_temp[1], &mag_temp[2]);
    //readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    //if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    //if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    delay(12); // 100 Hz ODR hardcoded
    }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    _magBias[0] = (float) mag_bias[0]* mRes * _magCalibration[0];  // save mag biases in G for main program
    _magBias[1] = (float) mag_bias[1]* mRes * _magCalibration[1];   
    _magBias[2] = (float) mag_bias[2]* mRes * _magCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    _magScale[0] = avg_rad/((float)mag_scale[0]);
    _magScale[1] = avg_rad/((float)mag_scale[1]);
    _magScale[2] = avg_rad/((float)mag_scale[2]);
  
   //Serial.println("Mag Calibration done!");
}
#endif
