
void dmpDataReady() {
	mpuInterrupt = true;
}

void setupIMU() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	//Uncomment if you want change i2c speed
	//if u not declared setClock, it set by default at 100KHz
	//Wire.setClock(400000); 

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	mpu.initialize();
	attachInterrupt(digitalPinToInterrupt(5), dmpDataReady, RISING); // i use pin 5 for external interrupt
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(12); //12
	mpu.setYGyroOffset(-18); //-18
	mpu.setZGyroOffset(41); //41
	mpu.setZAccelOffset(1009); //1009
	mpu.setYAccelOffset(-338); //-338
	mpu.setXAccelOffset(220); //220

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		attachInterrupt(digitalPinToInterrupt(DMP_INT), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		Serial.println("ERROR INITIALIZE!");
		while (1)
		{
		}
	}

	acc.timer.now = micros();
	acc.accX.now = 0;
	acc.accY.now = 0;
	acc.accZ.now = 0;

	acc.velX.now = 0;
	acc.velY.now = 0;
	acc.velZ.now = 0;

	acc.disX.now = 0;
	acc.disY.now = 0;
	acc.disZ.now = 0;

}

void readIMU() {
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	fifoCount = mpu.getFIFOCount();

	acc.timer.now = micros();
	acc.deltaTime = acc.timer.now - acc.timer.before;
	acc.timer.before = acc.timer.now;

	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

		yaw.now = yaw.before;
		pitch.now = pitch.before;
		roll.now = roll.before;
		Serial.println("ERROR OVERFLOW");
		mpu.resetFIFO();

	}
	else if (mpuIntStatus & 0x02) {
		if (fifoCount < packetSize) {}//i was changed this from original sample sketch
		else
		{
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;

			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			yaw.now = ypr[0] * 180 / M_PI;
			pitch.now = ypr[1] * 180 / M_PI;
			roll.now = ypr[2] * 180 / M_PI;

			yaw.before = yaw.now;
			pitch.before = pitch.now;
			roll.before = roll.now;


			tim.timer.now = micros();
			tim.deltaTime = tim.timer.now - tim.timer.before;
			tim.timer.before = tim.timer.now;

			// blink LED to indicate activity
			blinkState = !blinkState;
			digitalWrite(13, blinkState);
		}

	}

}