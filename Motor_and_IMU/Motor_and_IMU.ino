/* -----------------------------
 * ------- PREPROCESSOR --------
 * ----------------------------- */

#include <Wire.h>
#include <Kalman.h>
#include <QueueList.h>
#include <CommunicationUtils.h>
#include <DebugUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>

#define FORWARD 0
#define BACKWARD 1

#define R_MOTOR 0
#define L_MOTOR 1

/* -----------------------------
 * ----------- IMU -------------
 * ----------------------------- */

FreeSixIMU IMU = FreeSixIMU();
Kalman kalman;

float rawIMUValues[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // array passed to IMU object to be filled up with raw values
float zeroIMUAngle = 92.77; // 90 + 5 (offset from observations)

// sensor scale factors taken from TKJ Electronics' code
//const float GYRO_SCALE = 0.001009091;
const float GYRO_SCALE = 1;
const float ACC_SCALE = 0.1;

const float RADIAN_TO_DEGREE = float(180 / 3.14); // for use in accelerometer angle calculation

unsigned long lastTimeKalman = 0; // passsed into kalman filter

double pitch = 0; // stores filtered pitch of robot

/* -----------------------------
 * ---------- MOTOR ------------
 * ----------------------------- */

// motor PWM
const int L_MOTOR_PWM = 5;
const int R_MOTOR_PWM = 9;

// change direction of motors
const int L_MOTOR_IN_A = 3;
const int L_MOTOR_IN_B = 4;
const int R_MOTOR_IN_A = 11;
const int R_MOTOR_IN_B = 10;

const float DEADBAND_PITCH = 1.25;

/* -----------------------------
 * ----------- PID -------------
 * ----------------------------- */

// gains
const float kP_MAX = 70.0;
float kP = kP_MAX;
const float kI = 0.4;
const float kD = 0.5;

// to keep constant sample time
const int dt = 1; // sample time = 0.1 seconds
unsigned long nowTime = 0;
unsigned long lastTime = 0;
unsigned long timeChange = 0;

float setpoint = 0;
float command;

float currentError = 0.0;
float prevError = 0.0;

// for use in integral term calculation
float errorSum = 0.0;
QueueList <float> errorQueue;

/* -----------------------------
 * ------ COMPLEMENTARY --------
 * ----------------------------- */
unsigned long loopTime = 0;
const float COMPLEMENTARY_GAIN = 0.985;
float lastPitch = 0;
unsigned long lastStartTime = 0;

/* ====================================
 ================ SETUP ===============
 ====================================== */
void setup() {
  Serial.begin(9600);

  // Arduino pins to motor driver
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_IN_A, OUTPUT);
  pinMode(L_MOTOR_IN_B, OUTPUT);

  pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_IN_A, OUTPUT);
  pinMode(R_MOTOR_IN_B, OUTPUT);

  // IMU initialization
  Wire.begin(); // IMU connection
  delay(5);
  IMU.init(); // begin the IMU
  delay(5);
}

/* ====================================
 ================ LOOP ================
 ====================================== */
void loop() {
  nowTime = millis();
  loopTime = nowTime - lastStartTime;
  timeChange = nowTime - lastTime;

  // get raw acc and gyro readings
  updateIMU();

  // complementary filter
  pitch = COMPLEMENTARY_GAIN * (lastPitch + getGyroYRate() * loopTime / 1000) + (1 - COMPLEMENTARY_GAIN) * (getAccY() - zeroIMUAngle);
  lastPitch = pitch;
   
  if (timeChange >= dt) {
    // use pitch and PID controller to calculate motor command
    updateMotorsPID();
    lastTime = nowTime;
  }

  lastStartTime = nowTime;
}

/* ====================================
 ========== PID + MOVE MOTORS =========
 ====================================== */
void updateMotorsPID() {
  // calculate command
  currentError = pitch - setpoint;
  command = pTerm() + iTerm() + dTerm();

  // send command to motors
  int direction = (command >= 0) ? FORWARD : BACKWARD;
  int speed = min(abs(command), 255);
  
  moveMotor(R_MOTOR, direction, speed);
  moveMotor(L_MOTOR, direction, speed);
}

float pTerm() {
  // gain scheduling
//  float scale = abs(pitch)/DEADBAND_PITCH;
//  kP = min(scale * kP_MAX, kP_MAX);

  kP = min(abs(((50 * pow(pitch, 2)) + (30 * pitch))), kP_MAX);
  
  Serial.println(kP);
  return (kP * currentError);
}

float iTerm() {
  // calculate sum of last 100 errors
  errorQueue.push(currentError); // always add current error to stack
  errorSum += currentError;

  if (errorQueue.count() > 100) { // keeps most recent 100 values
    errorSum -= errorQueue.pop();
  }

  float iTerm = kI * errorSum;
  //  iTerm = constrain(iTerm, -90, 90) // integral limit to between -90 and 90
  return iTerm;
}

float dTerm() {
  float dTerm = kD * getGyroYRate();
//  prevError = currentError;
  return dTerm;
}

void moveMotor(int motor, int direction, int speed) {
  // PWM values: 25% = 64; 50% = 127; 75% = 191; 100% = 255
  if (motor == R_MOTOR) {
    analogWrite(R_MOTOR_PWM, speed);
    digitalWrite(R_MOTOR_IN_A, direction);
    digitalWrite(R_MOTOR_IN_B, abs(1 - direction));
  } else if (motor == L_MOTOR) {
    analogWrite(L_MOTOR_PWM, speed);
    digitalWrite(L_MOTOR_IN_A, direction);
    digitalWrite(L_MOTOR_IN_B, abs(1 - direction));
  }
}

/* ====================================
 ====== GET PROCESSED IMU VALUES ======
 ====================================== */
void updateIMU() {
  IMU.getValues(rawIMUValues);
}

// taken from TKJ Electronics' code
float getGyroYRate() {
  float rateY = (getRawGyroY() / GYRO_SCALE);

  return rateY;
}

// taken from TKJ Electronics' code
float getAccY() {
  float accXval = getRawAccX() / ACC_SCALE;
  float accYval = getRawAccY() / ACC_SCALE;
  accYval--; //-1g when lying down
  float accZval = getRawAccZ() / ACC_SCALE;

  float R = sqrt(pow(accXval, 2) + pow(accYval, 2) + pow(accZval, 2)); // Calculate the length of force vector
  float angleY = acos(accYval / R) * RADIAN_TO_DEGREE	;
  return angleY;
}

/* ====================================
 ========= GET RAW IMU VALUES =========
 ====================================== */
float getRawGyroY() {
  return rawIMUValues[3];
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
  Serial.print(rawIMUValues[3]); Serial.println('\t');
}
