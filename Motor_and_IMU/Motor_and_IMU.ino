/* -----------------------------
 * ----------- IMU ------------- 
 * ----------------------------- */
// libraries
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

FreeSixIMU IMU;
const int AvgAngles = 3;

/* UPDATE IMU ANGLE
 ---------------------------- */
float angles[5];
float currAngle, prevAngle;
float prevAngles[AvgAngles];
int prevAnglesArrayIndex = 0;
float anglesSum = 0;

/* CALIBRATE IMU ANGLE
 ---------------------------- */
float calibrateSum = 0;
float zeroAngle = 0;
const int CALIBRATE_NUM_TIMES = 200;

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

enum Direction {
  FORWARD,
  BACKWARD,
};

/* -----------------------------
 * ------ PID CONSTANTS -------- 
 * ----------------------------- */
const float kP = 10.0;
float setpoint;
float command;

/* -----------------------------
 * --- FUNCTION PROTOTYPES ----- 
 * ----------------------------- */
void calibrateIMU();
void updateAngle();
float readIMU_Y();
void moveMotors(Direction direction, int speed);
void updateMotorsPID();

void setup() {
  Serial.begin(9600);
  
  // Arduino pins to motor driver
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN_A, OUTPUT);
  pinMode(MOTOR_IN_B, OUTPUT);
  
  // IMU initialization
  Wire.begin(); // IMU connection

  IMU = FreeSixIMU();
  delay(5);
  IMU.init(); // Begin the IMU
  delay(5);

  calibrateIMU();

  setpoint = zeroAngle;
}

void loop() {
  updateAngle();
  updateMotorsPID();
  
  delay(10);
}

void updateMotorsPID() {
  command = kP * (currAngle - setpoint);
  Direction direction = (command >= 0) ? FORWARD : BACKWARD;
  int speed = min(abs(command), 255);
  Serial.println(command);
  moveMotors(direction, speed);
}

void moveMotors(Direction direction, int speed) {
  // PWM values: 25% = 64; 50% = 127; 75% = 191; 100% = 255
  analogWrite(MOTOR_PWM, speed);

  digitalWrite(MOTOR_IN_A, direction);
  digitalWrite(MOTOR_IN_B, abs(1 - direction));
}

void calibrateIMU() {
  for (int i = 0; i < CALIBRATE_NUM_TIMES; i++) {
    calibrateSum += readIMU_Y();
    Serial.println(readIMU_Y());
  }
  zeroAngle = calibrateSum / CALIBRATE_NUM_TIMES;

  Serial.println("--------------- CALIBRATED ---------------");
  Serial.println(zeroAngle);
  Serial.println("--------------- CALIBRATED ---------------");
}

void updateAngle() {
  prevAngles[prevAnglesArrayIndex] = readIMU_Y() - zeroAngle; // put calibrated pitch in prevAngles array
  prevAnglesArrayIndex = (prevAnglesArrayIndex + 1) % AvgAngles; // increment prevAnglesArrayIndex index in prevAngles array

  anglesSum = 0;
  for (int i = 0; i < AvgAngles; i++) // average all angles in prevAngles array
      anglesSum += prevAngles[i];
  currAngle = anglesSum / AvgAngles;
}

float readIMU_Y() {
  IMU.getYawPitchRoll(angles);
  return angles[2];
}
