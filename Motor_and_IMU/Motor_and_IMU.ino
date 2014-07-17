/* -----------------------------
 * ------- PREPROCESSOR --------
 * ----------------------------- */
#include <Wire.h>
#include <QueueList.h>
#include <CommunicationUtils.h>
#include <DebugUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>

// direction
#define FORWARD 0
#define BACKWARD 1

// motor
#define R_MOTOR 0
#define L_MOTOR 1

// encoder speed direction
#define ENC_FORWARD 1;
#define ENC_BACKWARD -1;

/* -----------------------------
 * ----------- IMU -------------
 * ----------------------------- */
FreeSixIMU IMU = FreeSixIMU();

float rawIMUValues[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // array passed to IMU object to be filled up with raw values
float zeroIMUAngle = 92.6; // 90 + offset from observations

// sensor scale factors
const float GYRO_SCALE = 1; // already taken care of in FreeSixIMU library
const float ACC_SCALE = 0.1; // taken from TKJ Electronics' code

const float RADIAN_TO_DEGREE = float(180 / 3.14); // for use in accelerometer angle calculation

double pitch = 0; // stores filtered pitch of robot

/* -----------------------------
 * ---------- MOTOR ------------
 * ----------------------------- */
// motor PWM
const int L_MOTOR_PWM = 6;
const int R_MOTOR_PWM = 9;

// change direction of motors
const int L_MOTOR_IN_A = 4;
const int L_MOTOR_IN_B = 5;
const int R_MOTOR_IN_A = 11;
const int R_MOTOR_IN_B = 10;

// deadband pitch
const float DEADBAND_PITCH = 1.25;

/* -----------------------------
 * --------- ENCODERS ----------
 * ----------------------------- */
// encoder Arduino ports
const int L_ENCODER_A = 2;
const int L_ENCODER_B = 7;
const int R_ENCODER_A = 3;
const int R_ENCODER_B = 8;

volatile long encoderRCount = 0;
volatile long encoderLCount = 0;

bool leftEncTimeState, rightEncTimeState = true;
long leftEnc_t1, rightEnc_t1 = 0;
long leftEnc_t2, rightEnc_t2 = 0;
long leftEnc_dt, rightEnc_dt = 0;

int leftEncDirection, rightEncDirection = ENC_FORWARD;

const int THRESHOLD_DT = 50; // number of milliseconds of not moving for speed to count as 0
const float MILLIS_TO_SEC = 1000;

/* -----------------------------
 * ---- SPEED TO ANGLE PID -----
 * ----------------------------- */
const float kP_speed = 0;

const float SPEED_SETPOINT = 0.0;

/* -----------------------------
 * ---- ANGLE TO MOTOR PID -----
 * ----------------------------- */
// gains
const float kP_MAX_ANGLE = 70.0;
float kP_angle = kP_MAX_ANGLE;
const float kI_angle = 0.4;
const float kD_angle = 0.5;

// to keep constant sample time
const int dt = 1; // sample time = 0.001 seconds
unsigned long nowTime = 0;
unsigned long lastTime = 0;
unsigned long timeChange = 0;

float angleSetpoint = 0.0;
float motorCommand;

float currentAngleError = 0.0;
float prevAngleError = 0.0;

// for use in integral term calculation
float angleErrorSum = 0.0;
QueueList <float> angleErrorQueue;

/* -----------------------------
 * --- COMPLEMENTARY FILTER ----
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

  // Arduino pins to encoder
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);

  // encoder read interrupts
  attachInterrupt(0, leftEncoder, RISING); // pin 2, low to high
  attachInterrupt(1, rightEncoder, RISING); // pin 3, low to high

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

//  Serial.println(pitch);

  if (timeChange >= dt) {
    // use pitch and PID controller to calculate motor motorCommand
    speedToAnglePID();
    angleToMotorPID();
    lastTime = nowTime;
  }

  lastStartTime = nowTime;
}

/* ====================================
 === SPEED TO ANGLE PID + GET SPEED ===
 ====================================== */
void speedToAnglePID() {
  Serial.print(getLeftSpeed()); Serial.print('\t'); Serial.println(getRightSpeed());
}

float getLeftSpeed() {
  if ((millis() - leftEnc_t2) < THRESHOLD_DT) {
    return leftEncDirection * (MILLIS_TO_SEC / leftEnc_dt);
  } else {
    return 0;
  }
}

float getRightSpeed() {
  if ((millis() - rightEnc_t2) < THRESHOLD_DT) {
    return rightEncDirection * (MILLIS_TO_SEC / rightEnc_dt);
  } else {
    return 0;
  }
}

/* ====================================
 == ANGLE TO MOTOR PID + MOVE MOTORS ==
 ====================================== */
void angleToMotorPID() {
  // calculate motorCommand
  currentAngleError = pitch - angleSetpoint;
  motorCommand = pTerm() + iTerm() + dTerm();

  // send motorCommand to motors
  int direction = (motorCommand >= 0) ? FORWARD : BACKWARD;
  int speed = min(abs(motorCommand), 255);

  moveMotor(R_MOTOR, direction, speed);
  moveMotor(L_MOTOR, direction, speed);
}

float pTerm() {
  // gain scheduling
  //  float scale = abs(pitch)/DEADBAND_PITCH;
  //  kP_angle = min(scale * kP_MAX_ANGLE, kP_MAX_ANGLE);

  kP_angle = min(abs(((55 * pow(pitch, 2)) + (20 * pitch) + 10)), kP_MAX_ANGLE);

  return (kP_angle * currentAngleError);
}

float iTerm() {
  // calculate sum of last 100 errors
  angleErrorQueue.push(currentAngleError); // always add current error to stack
  angleErrorSum += currentAngleError;

  if (angleErrorQueue.count() > 100) { // keeps most recent 100 values
    angleErrorSum -= angleErrorQueue.pop();
  }

  float iTerm = kI_angle * angleErrorSum;
  //  iTerm = constrain(iTerm, -90, 90) // integral limit to between -90 and 90
  return iTerm;
}

float dTerm() {
  float dTerm = kD_angle * (currentAngleError - prevAngleError);
  //  float dTerm = kD_angle * getGyroYRate();
  prevAngleError = currentAngleError;
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
 = ENCODER INTERRUPT SERVICE ROUTINES =
 ====================================== */
void leftEncoder() {
  // determine direction
  if (digitalRead(L_ENCODER_B) == LOW) {
    leftEncDirection = ENC_FORWARD;
  } else {
    leftEncDirection = ENC_BACKWARD;
  }
  
  // record dt time difference in milliseconds
  if (leftEncTimeState) {
    leftEnc_t1 = millis();
    leftEncTimeState = false;
  } else {
    leftEnc_t2 = millis();
    leftEnc_dt = leftEnc_t2 - leftEnc_t1;
    leftEncTimeState = true;
  }
}

void rightEncoder() {
  // determine direction
  if (digitalRead(R_ENCODER_B) == LOW) {
    rightEncDirection = ENC_FORWARD;
  } else {
    rightEncDirection = ENC_BACKWARD;
  }
  
  // record dt time difference in milliseconds
  if (rightEncTimeState) {
    rightEnc_t1 = millis();
    rightEncTimeState = false;
  } else {
    rightEnc_t2 = millis();
    rightEnc_dt = rightEnc_t1 - rightEnc_t2;
    rightEncTimeState = true;
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
  return (getRawGyroY() / GYRO_SCALE);
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
