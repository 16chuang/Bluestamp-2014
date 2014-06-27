// LIBRARIES
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

FreeSixIMU IMU = FreeSixIMU();

const int AvgAngles = 3;
// float prevTargetAngle = 0;
// float targetAngle = 0;


/* UPDATE IMU ANGLE
 ---------------------------- */
float angles[5];
float currAngle, prevAngle;
float prevAngles[AvgAngles];
int prevAngleI = 0;
float anglesSum = 0;

/* CALIBRATE IMU ANGLE
 ---------------------------- */
float calibrateSum = 0;
float zeroAngle = 0;
const int CALIBRATE_NUM_TIMES = 200;

// // time vars
// int currTime = 0; 
// int prevTime = 0; 

// float errorSum = 0;
// float currError = 0;
// float prevError = 0;
// float iTerm = 0;
// float dTerm = 0;
// float pTerm = 0;

// //Location PID CONTROL - These are the PID control for the robot trying to hold its location.
//   float Lp = 0.5;
//   float Li = 0.05;
//   float Ld = 0.4;
//   float offsetLoc = 0;
//   float pT,iT,dT = 0;
//   float errorS = 0;
//   float prevE = 0;

// FUNCTION PROTOTYPES
void calibrateIMU();
void updateAngle();
float readIMU_Y();

void setup() {
  // Serial Connection
  Serial.begin(9600);

  // IMU Connection
  Wire.begin(); 

  delay(5);
  IMU.init(); // Begin the IMU
  delay(5);
  
  calibrateIMU();
}

void loop() {
  updateAngle();
  Serial.println(currAngle);
  delay(100);
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
  prevAngles[prevAngleI] = readIMU_Y(); // put pitch in prevAngles array
  prevAngleI = (prevAngleI + 1) % AvgAngles; // increment prevAngleI index in prevAngles array

  anglesSum = 0;
  for (int i = 0; i < AvgAngles; i++) // average all angles in prevAngles array
      anglesSum += prevAngles[i];
  currAngle = anglesSum / AvgAngles;
  
  // prevAngle = currAngle; // unused???
}

float readIMU_Y() {
  IMU.getYawPitchRoll(angles);
  return angles[2];
}
