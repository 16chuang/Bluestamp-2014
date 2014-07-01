#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

FreeSixIMU IMU = FreeSixIMU();

float rawIMUValues[6];
float zeroIMUValues[4] = {0,0,0,0};

const float GYRO_SCALE = 0.001009091;
const float ACC_SCALE = 0.1;

const float RADIAN_TO_DEGREE = float(180 / 3.14);

const int CALIBRATE_NUM_TIMES = 100;

unsigned long lastTime = 0;

/* Kalman filter variables and constants */
const float Q_angle = 0.001; // Process noise covariance for the accelerometer - Sw
const float Q_gyro = 0.003; // Process noise covariance for the gyro - Sw
const float R_angle = 0.03; // Measurement noise covariance - Sv

double angle = 180; // It starts at 180 degrees
double bias = 0;
double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
double dt, y, S;
double K_0, K_1;

double pitch = 0;

/* ====================================
 ================ SETUP ===============
 ====================================== */
void setup() {
  // Serial Connection
  Serial.begin(9600);

  // IMU Connection
  Wire.begin(); 

	// Begin the IMU
  delay(5);
  IMU.init(); 
  delay(5);

  calibrateIMU();
}

void loop() {
  updateIMU();

  pitch = kalman(double(getAccY()), double(getGyroYRate()), millis() - lastTime);
  Serial.println(pitch);
  lastTime = millis();
}

/* ====================================
 ============ KALMAN FILTER ===========
 ====================================== */
double kalman(double newAngle, double newRate, double dtime) {
	dt = dtime / 1000000; // Convert from microseconds to seconds

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	angle += dt * (newRate - bias);

	// Update estimation error covariance - Project the error covariance ahead
	P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
	P_01 += -dt * P_11;
	P_10 += -dt * P_11;
	P_11 += +Q_gyro * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	S = P_00 + R_angle;
	K_0 = P_00 / S;
	K_1 = P_10 / S;

	// Calculate angle and resting rate - Update estimate with measurement zk
	y = newAngle - angle;
	angle += K_0 * y;
	bias += K_1 * y;

	// Calculate estimation error covariance - Update the error covariance
	P_00 -= K_0 * P_00;
	P_01 -= K_0 * P_01;
	P_10 -= K_1 * P_00;
	P_11 -= K_1 * P_01;
                
	return angle;
}

/* ====================================
 ====== GET PROCESSED IMU VALUES ======
 ====================================== */
void updateIMU() {
	IMU.getValues(rawIMUValues);
	getGyroYRate();
	getAccY();
}

float getGyroYRate() {
	// (gyroAdc-gyroZero)/Sensitivity (In quids) - Sensitivity = 0.00333/3.3=0.001009091
	// (gyroAdc - gyroZero) * scale
	float rateY = ((getRawGyroY() - zeroIMUValues[3]) / GYRO_SCALE);
//        Serial.print(rateY); Serial.print('\t');
        return rateY;
}

float getAccY() {
	float accXval = (getRawAccX() - zeroIMUValues[0]) / ACC_SCALE;
	float accYval = (getRawAccY() - zeroIMUValues[1]) / ACC_SCALE;
	accYval--; //-1g when lying down
	float accZval = (getRawAccZ() - zeroIMUValues[2]) / ACC_SCALE;

	float R = sqrt(pow(accXval, 2) + pow(accYval, 2) + pow(accZval, 2)); // Calculate the length of force vector
	float angleY = acos(accYval / R) * RADIAN_TO_DEGREE	;

//        Serial.print(angleY); Serial.print('\n');
	return angleY;
}

/* ====================================
 ============ CALIBRATION =============
 ====================================== */
void calibrateIMU() {
	// add CALIBRATE_NUM_TIMES
	for (int i = 0; i < CALIBRATE_NUM_TIMES; i++) {
		IMU.getValues(rawIMUValues);
		zeroIMUValues[0] += getRawAccX();
		zeroIMUValues[1] += getRawAccY();
		zeroIMUValues[2] += getRawAccZ();
		zeroIMUValues[3] += getRawGyroY();
//                printRawIMUValues();
	}

	// average
	for (int i = 0; i < 4; i++) {
		zeroIMUValues[i] /= CALIBRATE_NUM_TIMES;
      Serial.println("----------------calibrated-----------------");
      Serial.println(zeroIMUValues[i]);
	}
}

/* ====================================
 ========= GET RAW IMU VALUES =========
 ====================================== */
float getRawGyroY() {
  return rawIMUValues[4];
}

float getRawAccX() {
	return rawIMUValues[0];
}

float getRawAccY() {
	return rawIMUValues[1];
}

float getRawAccZ() {
	return rawIMUValues[2];
}

void printRawIMUValues() {
	Serial.print(rawIMUValues[0]); Serial.print('\t');
	Serial.print(rawIMUValues[1]); Serial.print('\t');
	Serial.print(rawIMUValues[2]); Serial.print('\t');
	Serial.print(rawIMUValues[3]); Serial.print('\n');
}
