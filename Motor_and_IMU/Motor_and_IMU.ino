/* -----------------------------
 * ------- PREPROCESSOR -------- 
 * ----------------------------- */

#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Kalman.h>
#include <QueueList.h>

#define FORWARD 0
#define BACKWARD 1

/* -----------------------------
 * ----------- IMU ------------- 
 * ----------------------------- */

FreeSixIMU IMU = FreeSixIMU();
Kalman kalman;

float rawIMUValues[6] = {0, 0, 0, 0, 0, 0}; // array passed to IMU object to be filled up with raw values
int zeroIMUAngle = 95; // 90 + 5 (offset from observations)

// sensor scale factors taken from TKJ Electronics' code
const float GYRO_SCALE = 0.001009091;
const float ACC_SCALE = 0.1;

const float RADIAN_TO_DEGREE = float(180 / 3.14); // for use in accelerometer angle calculation

unsigned long lastTime = 0; // passsed into kalman filter

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

/* -----------------------------
 * ----------- PID ------------- 
 * ----------------------------- */

// gains
const float kP = 25.0;
const float kI = 0.01;
const float kD = 0.0;

float setpoint = 0;
float command;

float currentError = 0.0;
float prevError = 0.0;

// for use in integral term calculation
float errorSum = 0.0;
QueueList <float> errorQueue; 

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
  
  // read and get rid of the first IMU value (b/c huge, ugly, and random...)
  pitch = kalman.getAngle(double(getAccY()), double(getGyroYRate()), double((micros() - lastTime) / 1000)) - zeroIMUAngle;
  Serial.print("FIRST KALMAN PITCH: "); Serial.println(pitch);
}

/* ====================================
 ================ LOOP ================
 ====================================== */
void loop() {  
  unsigned long loopStart = millis();
  
  // get raw acc and gyro readings
  updateIMU();

  // filter readings to get pitch
  pitch = kalman.getAngle(double(getAccY()), double(getGyroYRate()), double((micros() - lastTime) / 1000)) - zeroIMUAngle;
  lastTime = micros();
  
  // use pitch and PID controller to calculate motor command
  updateMotorsPID();
  
  // print loop time
  unsigned long loopEnd = millis() - loopStart;
  Serial.print("loop end "); Serial.println(loopEnd);
}

/* ====================================
 ========== PID + MOVE MOTORS =========
 ====================================== */
void updateMotorsPID() {
  // calculate command
  currentError = pitch - setpoint;
  command = pTerm() + iTerm();
  
  Serial.print("COMMAND: "); Serial.print(command);
  Serial.print('\t');
  
  // send command to motors
  int direction = (command >= 0) ? FORWARD : BACKWARD;
  int speed = min(abs(command), 255);
  moveMotors(direction, speed);
}

float pTerm() {
  return (kP * currentError);
}

float iTerm() {
  // calculate sum of last 100 errors
  errorQueue.push(currentError); // always add current error to stack
  errorSum += currentError;
  
  if (errorQueue.count() > 100) { // keeps most recent 100 values
    errorSum -= errorQueue.peek();
    Serial.print("popping "); Serial.print(errorQueue.peek()); Serial.print('\t');
    errorQueue.pop();
  }
  
  Serial.print("ERROR SUM: "); Serial.print(errorSum); 
  Serial.print('\t'); 
  Serial.print("CURRENT ERROR: "); Serial.print(currentError);
  Serial.print('\t'); 
  
  float iTerm = kI * errorSum;
//  iTerm = min(max(iTerm, -90), 90); // integral limit to between -90 and 90
  return iTerm;
}

float dTerm() {
  float dTerm = kD * (currentError - prevError);
  prevError = currentError;
  return dTerm;
}

void moveMotors(int direction, int speed) {
  // PWM values: 25% = 64; 50% = 127; 75% = 191; 100% = 255
  analogWrite(L_MOTOR_PWM, speed);
  analogWrite(R_MOTOR_PWM, speed);

  // direction
  digitalWrite(L_MOTOR_IN_A, direction);
  digitalWrite(L_MOTOR_IN_B, abs(1 - direction));
  digitalWrite(R_MOTOR_IN_A, direction);
  digitalWrite(R_MOTOR_IN_B, abs(1 - direction));
}

/* ====================================
 ====== GET PROCESSED IMU VALUES ======
 ====================================== */
void updateIMU() {
  IMU.getValues(rawIMUValues);
}

// taken from TKJ Electronics' code
float getGyroYRate() {
  // (gyroAdc-gyroZero)/Sensitivity (In quids) - Sensitivity = 0.00333/3.3=0.001009091
  // (gyroAdc - gyroZero) * scale
  float rateY = (getRawGyroY() / GYRO_SCALE);
  
//  Serial.print(rateY); Serial.print('\t');
  
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

//  Serial.print(angleY); Serial.print('\t');
  
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
  Serial.print(rawIMUValues[4]); Serial.print('\t');
}
