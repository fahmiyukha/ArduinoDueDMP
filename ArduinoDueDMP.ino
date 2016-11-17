/*
*Author : Fahmi Yukha S.
*Device : Arduino DUe
*/

/************************************HEADER***********************************/

//MPU6050DMP
#include "I2Cdev.h"
#include "MPU6050\MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

/************************************HEADER***********************************/

/************************************MACRO***********************************/
//MPU6050DMP
#define LED_PIN 13 
#define DMP_INT 5

#define NOW 0
#define BEFORE 1

#define PTC 0
#define RLL 1
#define THR 2
#define YAW 3
/************************************MACRO***********************************/


/**********************************REFERENCE**********************************/
//MPU6050DMP
MPU6050 mpu;

struct dataF {
	float now;
	float before;
};

struct dataD {
	double now;
	double before;
};

struct dataI {
	int16_t now;
	int16_t before;
};

struct dataU {
	uint16_t now;
	uint16_t before;
};

struct dataL{
	long now;
	long before;
};

struct accelRaw {
	dataF accX;
	dataF accY;
	dataF accZ;
	dataF velX;
	dataF velY;
	dataF velZ;
	dataF disX;
	dataF disY;
	dataF disZ;
	dataL timer;
	long deltaTime;
	double deltaSec;
};

accelRaw acc, tim;
/**********************************REFERENCE**********************************/

/**********************************VARIABLE***********************************/

//MPU6050DMP
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

dataF yaw;
dataF pitch, pitchKal;
dataF roll, rollKal;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

int16_t ax, ay, az;
int16_t gx, gy, gz;


//Telemetry
unsigned long timerPrintOut;

/**********************************VARIABLE***********************************/

void setup()
{
	Serial.begin(115200);
	setupIMU();
	pinMode(13, OUTPUT);
	/* add setup code here */

}

void loop()
{

	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //Read Raw data from MPU6050
	readIMU();
	if ((millis() - timerPrintOut) > 100) // Print every 100ms
	{
		timerPrintOut = millis();
		Serial.print(" |P: ");		Serial.print(pitch.now);
		Serial.print(" R: ");		Serial.print(roll.now);
		Serial.print(" | ");		Serial.print(acc.deltaTime); //Timer for delta time loop
		Serial.print(" | ");		Serial.print(tim.deltaTime); //Timer for delta time DMP

		Serial.println();
	}
	/* add main program code here */

}
