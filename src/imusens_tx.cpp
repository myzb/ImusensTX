// Do not remove the include below
#include "imusens_tx.h"

//#define SERIAL_DEBUG
#define FUSION

#include "MPU9250.h"


typedef union {
  float num;
  byte raw[4];
} data_t;

// MPU9250 vars
const int gIntPin = 33; // INT connected to pin 33
const int gLedPin = 13; // LED connected to pin 13
int       gBeginStatus, gSetFiltStatus;
float     imuData[10];  // ax, ay, az, gx, gy, gz, hx, hy, hz, t
volatile int i = 0;

// RawHID vars
byte buffer[64];        // RawHID packets are always 64 bytes
elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;

// MPU9250 object with its I2C address of 0x68
// (ADDR to GRND) and on Teensy bus 0
MPU9250 imu(0x68, 0);

//The setup function is called once at startup of the sketch
void setup()
{
	  // Begin serial output
	  Serial.begin(115200);

	  // Enable led
	  pinMode(gLedPin, OUTPUT);
	  digitalWrite(gLedPin, HIGH);

	  // start communication with IMU and
	  // set the accelerometer and gyro ranges.
	  // ACCELEROMETER 2G 4G 8G 16G
	  // GYRO 250DPS 500DPS 1000DPS 2000DPS
	  gBeginStatus = imu.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);

	  //imu.selfTest(); // Start by performing self test and reporting values
	  //imu.accelGyroCal();
	  //imu.magCal();

	  if(gBeginStatus < 0) {
	    delay(1000);
	#ifdef SERIAL_DEBUG
	    Serial.println("IMU initialization unsuccessful");
	    Serial.println("Check IMU wiring or try cycling power");
	#endif
	    while(1){};
	  }

	  // set up the IMU DLPF, data output rate,
	  // and interrupt. DLPF set to 41 Hz,
	  // data output rate set to 100 Hz, and
	  // MPU-9250 generated interrupt attached
	  // to Teensy pin 2
	  gSetFiltStatus = imu.setFilt(DLPF_BANDWIDTH_41HZ,9);
	  //gSetFiltStatus = imu.enableInt(true);
	  if(gSetFiltStatus < 0) {
	    delay(1000);
	    Serial.println("Filter initialization unsuccessful");
	    while(1){};
	  } else {
	#ifdef SERIAL_DEBUG
	    Serial.println("Filter initialized");
	    // nada
	#endif
	  }
	  pinMode(gIntPin,INPUT);
	  attachInterrupt(gIntPin,getIMU,RISING);
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
}

void getMsg() {
  int n;
  n = RawHID.recv(buffer, 0); // 0 timeout = do not wait
  if (n > 0) {
    // the computer sent a message.  Display the bits
    // of the first byte on pin 0 to 7.  Ignore the
    // other 63 bytes!
    Serial.print(F("Received packet, first byte: "));
    Serial.println((int)buffer[0]);
    for (int i=0; i<8; i++) {
      int b = buffer[0] & (1 << i);
      digitalWrite(i, b);
    }
  }
}

void printData(){
  int n;
  data_t reading;

#ifdef SERIAL_DEBUG
  for (int i = 0; i < 10; i++) {
    Serial.print(imuData[i],6);
    Serial.print("\t");
  }
  Serial.print("\n");
#endif /* SERIAL_DEBUG */

  // every 2 seconds, send a packet to the computer
  //if (msUntilNextSend > 2000) {
    msUntilNextSend = msUntilNextSend - 2000;
    // first 2 bytes are a signature
    //buffer[0] = 0xAB;
    //buffer[1] = 0xCD;
    // next 24 bytes are analog measurements
    for (int i=0; i<10; i++) {
      reading.num = imuData[i];
      buffer[4*i] = reading.raw[0];
      buffer[4*i+1] = reading.raw[1];
      buffer[4*i+2] = reading.raw[2];
      buffer[4*i+3] = reading.raw[3];
    }
    // fill the rest with zeros
    for (int i=40; i<62; i++) {
      buffer[i] = 0;
    }
    // and put a count of packets sent at the end
    buffer[62] = highByte(packetCount);
    buffer[63] = lowByte(packetCount);
    // actually send the packet
    n = RawHID.send(buffer, 10);

    if (n > 0) {
#ifdef SERIAL_DEBUG
      Serial.print(F("Transmit packet "));
      Serial.println(packetCount);
#endif /* SERIAL_DEBUG */
      packetCount = packetCount + 1;
      // Toggle led
      digitalWrite(gLedPin, !digitalRead(gLedPin));
    } else {
#ifdef SERIAL_DEBUG
      Serial.println(F("Unable to transmit packet"));
#endif
    //}
  }

//  Serial.print(sizeof(int),6);
//  Serial.print("\t");
//  Serial.print(sizeof(float),6);
//  Serial.print("\t");
//  Serial.print(sizeof(double),6);
//  Serial.print("\t");

//  // print the data
//  Serial.print(ax,6);
//  Serial.print("\t");
//  Serial.print(ay,6);
//  Serial.print("\t");
//  Serial.print(az,6);
//  Serial.print("\t");
//
//  Serial.print(gx,6);
//  Serial.print("\t");
//  Serial.print(gy,6);
//  Serial.print("\t");
//  Serial.print(gz,6);
//  Serial.print("\t");
//
//  Serial.print(hx,6);
//  Serial.print("\t");
//  Serial.print(hy,6);
//  Serial.print("\t");
//  Serial.print(hz,6);
//  Serial.print("\t");
//
//  Serial.println(t,6);
}

void getIMU(){
  // get the accel (m/s/s), gyro (rad/s), and magnetometer (uT), and temperature (C) data
  imu.getMotion10(&imuData[0], &imuData[1], &imuData[2],
                  &imuData[3], &imuData[4], &imuData[5],
                  &imuData[6], &imuData[7], &imuData[8],
                  &imuData[9]);
  i++;
  // print the results every 10 frames
  if(i > 0){
    printData();
    i = 0;
  }
}
