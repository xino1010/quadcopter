#include "MPU9250.h"

#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#define AK8963_WHO_AM_I  0x00 // should return 0x48

// Pin definitions
#define INT_PIN 12
#define LED_PIN 13

MPU9250 mpu9250;
uint32_t Now = 0;        // used to calculate integration interval
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0; // used to calculate integration interval
float temperature;    // Stores the real internal chip temperature in degrees Celsius
float accelerations[3] = {0.0, 0.0, 0.0};
float gyrometers[3] = {0.0, 0.0, 0.0};
float magnetometers[3] = {0.0, 0.0, 0.0};

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

		mpu9250.MPU9250SelfTest(); // Start by performing self test and reporting values
		mpu9250.calibrateMPU9250(); // Calibrate gyro and accelerometers, load biases in bias registers
		delay(1000);

		mpu9250.initMPU9250();
		Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of accelerometer, gyroscope, and temperature

		// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
		byte d = mpu9250.readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
		Serial.print("AK8963 I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
		delay(1000); 

		// Get magnetometer calibration from AK8963 ROM
		mpu9250.initAK8963();
		Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
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
		mpu9250.readAccelData(accelerations);
		mpu9250.readGyroData(gyrometers);
		mpu9250.readMagData(magnetometers);
	}

	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;

	sum += deltat; // sum for averaging filter update rate
	sumCount++;

	//  mpu9250.MadgwickQuaternionUpdate(accelerations[0], accelerations[1], accelerations[3], gyrometers[0]*PI/180.0f, gyrometers[1] * PI/180.0f, gyrometers[2] * PI/180.0f,  magnetometers[1], magnetometers[0], magnetometers[2], deltat);
	mpu9250.MahonyQuaternionUpdate(accelerations[0], accelerations[1], accelerations[2], gyrometers[0] * PI/180.0f, gyrometers[1] * PI/180.0f, gyrometers[2] * PI/180.0f, magnetometers[1], magnetometers[0], magnetometers[2], deltat);

	if (!AHRS) {
		delt_t = millis() - count;
		if (delt_t > 500) {
			if (SerialDebug) {
				// Print acceleration values in milligs!
				Serial.print("X-acceleration: "); Serial.print(1000 * accelerations[0]); Serial.print(" mg ");
				Serial.print("Y-acceleration: "); Serial.print(1000 * accelerations[1]); Serial.print(" mg ");
				Serial.print("Z-acceleration: "); Serial.print(1000 * accelerations[2]); Serial.println(" mg ");

				// Print gyro values in degree/sec
				Serial.print("X-gyro rate: "); Serial.print(gyrometers[0], 3); Serial.print(" degrees/sec "); 
				Serial.print("Y-gyro rate: "); Serial.print(gyrometers[1], 3); Serial.print(" degrees/sec "); 
				Serial.print("Z-gyro rate: "); Serial.print(gyrometers[2], 3); Serial.println(" degrees/sec"); 

				// Print mag values in degree/sec
				Serial.print("X-mag field: "); Serial.print(magnetometers[0]); Serial.print(" mG "); 
				Serial.print("Y-mag field: "); Serial.print(magnetometers[1]); Serial.print(" mG "); 
				Serial.print("Z-mag field: "); Serial.print(magnetometers[2]); Serial.println(" mG"); 

				temperature = mpu9250.readTempData();  // Read the adc values
				// Print temperature in degrees Centigrade      
				Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
			}

			count = millis();
			digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // toggle led
		}
	}
	else {
		delt_t = millis() - count;
		if (delt_t > 500) {
			if (SerialDebug) {
				Serial.print("Yaw, Pitch, Roll: ");
				Serial.print(mpu9250.getYaw(), 2);
				Serial.print(", ");
				Serial.print(mpu9250.getPitch(), 2);
				Serial.print(", ");
				Serial.println(mpu9250.getRoll(), 2);
				Serial.print("rate = "); Serial.print((float) sumCount / sum, 2); Serial.println(" Hz");
			}
			count = millis(); 
			sumCount = 0;
			sum = 0;    
		}
	}
}
