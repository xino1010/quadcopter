#include "MPU9250.h"

#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#define AK8963_WHO_AM_I  0x00 // should return 0x48

// Pin definitions
#define INT_PIN 12
#define LED_PIN 13

MPU9250 mpu9250;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
float selfTest[6];    // holds results of gyro and accelerometer self test
uint32_t Now = 0;        // used to calculate integration interval
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float pitch, yaw, roll;
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
int16_t tempCount;      // temperature raw count output
float temperature;    // Stores the real internal chip temperature in degrees Celsius
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

void setup() {
	Wire.begin();
  	//  TWBR = 12;  // 400 kbit/sec I2C speed
	Serial.begin(38400);

	// Set up the interrupt pin, its set as active high, push-pull
	pinMode(INT_PIN, INPUT);
	digitalWrite(INT_PIN, LOW);
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	// Read the WHO_AM_I register, this is a good test of communication
  	byte c = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  	Serial.print("MPU9250 I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

  	// WHO_AM_I should always be 0x68
  	if (c == 0x71) {  
		Serial.println("MPU9250 is online...");

		mpu9250.MPU9250SelfTest(selfTest); // Start by performing self test and reporting values
		if (SerialDebug) {
			Serial.print("x-axis self test: acceleration trim within : "); Serial.print(selfTest[0],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: acceleration trim within : "); Serial.print(selfTest[1],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: acceleration trim within : "); Serial.print(selfTest[2],1); Serial.println("% of factory value");
			Serial.print("x-axis self test: gyration trim within : "); Serial.print(selfTest[3],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: gyration trim within : "); Serial.print(selfTest[4],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: gyration trim within : "); Serial.print(selfTest[5],1); Serial.println("% of factory value");
		}

		mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
		if (SerialDebug) {
			Serial.println("MPU9250 bias");
			Serial.print("x: ");
			Serial.print((int)(1000 * accelBias[0]));
			Serial.print(" y: ");
	    	Serial.print((int)(1000 * accelBias[1]));
			Serial.print(" z: ");
	    	Serial.print((int)(1000 * accelBias[2]));
	    	Serial.println(" mg");

	    	Serial.print("x: ");
			Serial.print(gyroBias[0]);
			Serial.print(" y: ");
			Serial.print(gyroBias[1]);
			Serial.print(" z: ");
			Serial.print(gyroBias[2]);
	    	Serial.println(" o/s");
	    }
		delay(1000);

		mpu9250.initMPU9250();
		Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of accelerometer, gyroscope, and temperature

		// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
		byte d = mpu9250.readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
		Serial.print("AK8963 I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
		delay(1000); 

		// Get magnetometer calibration from AK8963 ROM
		mpu9250.initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

		if (SerialDebug) {
			Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
			Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
			Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
		}

    	delay(1000);  
  	}
  	else {
		Serial.print("Could not connect to MPU9250: 0x");
	    Serial.println(c, HEX);
	    while(1) ; // Loop forever if communication doesn't happen
  	}	
}

void loop() {
	if (mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
		mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
		float aRes = mpu9250.getAres();

		// Now we'll calculate the accleration value into actual g's
		ax = (float) accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
		ay = (float) accelCount[1] * aRes; // - accelBias[1];   
		az = (float) accelCount[2] * aRes; // - accelBias[2];  

		mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
		float gRes = mpu9250.getGres();

		// Calculate the gyro value into actual degrees per second
		gx = (float) gyroCount[0] * gRes;  // get actual gyro value, this depends on scale being set
		gy = (float) gyroCount[1] * gRes;  
		gz = (float) gyroCount[2] * gRes;   

		mpu9250.readMagData(magCount);  // Read the x/y/z adc values
		float mRes = mpu9250.getMres();
		magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
		magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
		magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx = (float) magCount[0] * mRes * magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
		my = (float) magCount[1] * mRes * magCalibration[1] - magbias[1];  
		mz = (float) magCount[2] * mRes * magCalibration[2] - magbias[2];   
	}

	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;

	sum += deltat; // sum for averaging filter update rate
	sumCount++;

	//  mpu9250.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz, deltat, q);
	mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz, deltat, q, eInt);

	if (!AHRS) {
		delt_t = millis() - count;
		if (delt_t > 500) {
			if (SerialDebug) {
				// Print acceleration values in milligs!
				Serial.print("X-acceleration: "); Serial.print(1000 * ax); Serial.print(" mg ");
				Serial.print("Y-acceleration: "); Serial.print(1000 * ay); Serial.print(" mg ");
				Serial.print("Z-acceleration: "); Serial.print(1000 * az); Serial.println(" mg ");

				// Print gyro values in degree/sec
				Serial.print("X-gyro rate: "); Serial.print(gx, 3); Serial.print(" degrees/sec "); 
				Serial.print("Y-gyro rate: "); Serial.print(gy, 3); Serial.print(" degrees/sec "); 
				Serial.print("Z-gyro rate: "); Serial.print(gz, 3); Serial.println(" degrees/sec"); 

				// Print mag values in degree/sec
				Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG "); 
				Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG "); 
				Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG"); 

				tempCount = mpu9250.readTempData();  // Read the adc values
				temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
				// Print temperature in degrees Centigrade      
				Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
			}

			count = millis();
			digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // toggle led
		}
	}
	else {
		delt_t = millis() - count;
		if (delt_t > 500) { // update LCD once per half-second independent of read rate

			if (SerialDebug) {
				Serial.print("ax = "); Serial.print((int) 1000 * ax);  
				Serial.print(" ay = "); Serial.print((int) 1000 * ay); 
				Serial.print(" az = "); Serial.print((int) 1000 * az); Serial.println(" mg");
				Serial.print("gx = "); Serial.print(gx, 2);
				Serial.print(" gy = "); Serial.print(gy, 2); 
				Serial.print(" gz = "); Serial.print(gz, 2); Serial.println(" deg/s");
				Serial.print("mx = "); Serial.print((int) mx); 
				Serial.print(" my = "); Serial.print((int) my); 
				Serial.print(" mz = "); Serial.print((int) mz); Serial.println(" mG");

				Serial.print("q0 = "); Serial.print(q[0]);
				Serial.print(" qx = "); Serial.print(q[1]); 
				Serial.print(" qy = "); Serial.print(q[2]); 
				Serial.print(" qz = "); Serial.println(q[3]); 
			}               

			// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
			// In this coordinate system, the positive z-axis is down toward Earth. 
			// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
			// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
			// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
			// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
			// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
			// applied in the correct order which for this configuration is yaw, pitch, and then roll.
			// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
			yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
			pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
			roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
			pitch *= 180.0f / PI;
			yaw   *= 180.0f / PI; 
			yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
			roll  *= 180.0f / PI;

			if (SerialDebug) {
				Serial.print("Yaw, Pitch, Roll: ");
				Serial.print(yaw, 2);
				Serial.print(", ");
				Serial.print(pitch, 2);
				Serial.print(", ");
				Serial.println(roll, 2);
				Serial.print("rate = "); Serial.print((float) sumCount / sum, 2); Serial.println(" Hz");
			}

			count = millis(); 
			sumCount = 0;
			sum = 0;    
		}
	}
}
