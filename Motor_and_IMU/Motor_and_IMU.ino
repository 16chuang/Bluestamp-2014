/* -----------------------------
 * ------- PREPROCESSOR -------- 
 * ----------------------------- */

#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Kalman.h>

#define FORWARD 0
#define BACKWARD 1

/* -----------------------------
 * ----------- IMU ------------- 
 * ----------------------------- */

FreeSixIMU IMU = FreeSixIMU();
Kalman kalman;

float rawIMUValues[6] = {0, 0, 0, 0, 0, 0};
int zeroIMUAngle = 90;

const float GYRO_SCALE = 0.001009091;
const float ACC_SCALE = 0.1;

const float RADIAN_TO_DEGREE = float(180 / 3.14);

const int CALIBRATE_NUM_TIMES = 200;

unsigned long lastTime = 0;

double pitch = 0;

/* -----------------------------
 * ---------- MOTOR ------------ 
 * ----------------------------- */

// motor PWM 
const int MOTOR_PWM = 9; 

// change direction of motors
// A = 1, B = 0 -- clockwise
// A = 0, B = 1 -- counterclockwise
const int MOTOR_IN_A = 10;
const int MOTOR_IN_B = 11;



/* -----------------------------
 * ------ PID CONSTANTS -------- 
 * ----------------------------- */
const float kP = 10.0;
float setpoint = 0;
float command;

/* ====================================
 ================ SETUP ===============
 ====================================== */
void setup() {
  Serial.begin(9600);
  
  // Arduino pins to motor driver
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN_A, OUTPUT);
  pinMode(MOTOR_IN_B, OUTPUT);
  
  // IMU initialization
  Wire.begin(); // IMU connection

  delay(5);
  IMU.init(); // begin the IMU
  delay(5);
}

void loop() {
  updateIMU();

  pitch = kalman.getAngle(double(getAccY()), double(getGyroYRate()), double((micros() - lastTime) / 1000)) - zeroIMUAngle;
  Serial.println(pitch);
  lastTime = micros();
  
  updateMotorsPID();
  
//  delay(10);
}

/* ====================================
 ============= MOVE MOTORS ============
 ====================================== */
void updateMotorsPID() {
  command = kP * (pitch - setpoint);
  int direction = (command >= 0) ? FORWARD : BACKWARD;
  int speed = min(abs(command), 255);
//  Serial.println(command);
  moveMotors(direction, speed);
}



void moveMotors(int direction, int speed) {
  // PWM values: 25% = 64; 50% = 127; 75% = 191; 100% = 255
  analogWrite(MOTOR_PWM, speed);

  digitalWrite(MOTOR_IN_A, direction);
  digitalWrite(MOTOR_IN_B, abs(1 - direction));
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
