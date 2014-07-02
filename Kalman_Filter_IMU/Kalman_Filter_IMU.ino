#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Kalman.h>

FreeSixIMU IMU = FreeSixIMU();
Kalman kalman;

float rawIMUValues[6];
int zeroIMUAngle = 90;

const float GYRO_SCALE = 0.001009091;
const float ACC_SCALE = 0.1;

const float RADIAN_TO_DEGREE = float(180 / 3.14);

const int CALIBRATE_NUM_TIMES = 200;

unsigned long lastTime = 0;

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
}

void loop() {
  updateIMU();

  pitch = kalman.getAngle(double(getAccY()), double(getGyroYRate()), double((micros() - lastTime) / 1000)) - zeroIMUAngle;
  Serial.println(pitch);
  lastTime = micros();
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
  float rateY = (getRawGyroY() / GYRO_SCALE);
  //        Serial.print(rateY); Serial.print('\t');
  return rateY;
}

float getAccY() {
  float accXval = getRawAccX() / ACC_SCALE;
  float accYval = getRawAccY() / ACC_SCALE;
  accYval--; //-1g when lying down
  float accZval = getRawAccZ() / ACC_SCALE;

  float R = sqrt(pow(accXval, 2) + pow(accYval, 2) + pow(accZval, 2)); // Calculate the length of force vector
  float angleY = acos(accYval / R) * RADIAN_TO_DEGREE	;

  //        Serial.print(angleY); Serial.print('\n');
  return angleY;
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
