//John Dingley 08/03/2014   IMU TESTING PROFRAM
//This program modified slightly from Piddybot program see below. If you wire up the Sparkfun Digital IMU and connect
//the USB lead to your computer and open the Serial View window (9600 Baud)
//then when you move the IMU from the flat position you will see the displayed "angle" varies from zero when level
//through -ve values when tilted one way to +ve similar sized values when tilted the other way.

//This code therefore allows you to test your IMU is working OK and talking to the Arduino before attaching the Sabertooth power controller
//or attaching the deadman switch, steering rocker switch or the balance point fine tuning rocker switch.

// Modified from PIDDYBOT Self Balancing Program
// Program Written and Pieced together by Sean Hodgins.
// The program was morphed from many programs.
// With that being said If you feel you see something that
// is your work and want credit, feel free to contact me.
// Http://Idlehandsproject.com
// This is free to be shared, altered, and used. 
// It is in no way "Finished".
// Find the Second target angle and tune for your bot, it may be different.
// LIBRARIES
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

FreeSixIMU sixDOF = FreeSixIMU();


const int AvgAngles = 3;
 float prevTargetAngle = 0;
 float targetAngle = 0;


float angles[5];

float currAngle, prevAngle;
float prevAngles[AvgAngles];
int prevAngleI = 0;

// time vars
int currTime = 0; 
int prevTime = 0; 



float errorSum = 0;
float currError = 0;
float prevError = 0;
float iTerm = 0;
float dTerm = 0;
float pTerm = 0;

//Location PID CONTROL - These are the PID control for the robot trying to hold its location.
  float Lp = 0.5;
  float Li = 0.05;
  float Ld = 0.4;
  float offsetLoc = 0;
  float pT,iT,dT = 0;
  float errorS = 0;
  float prevE = 0;

void setup() {

  
  // Serial Connection
  Serial.begin(9600);

  // IMU Connection
  Wire.begin(); 

  delay(5);
  sixDOF.init(); //Begin the IMU
  delay(5);

}


void loop() {

  updateAngle();
  Serial.println(currAngle);
  delay(500);
  
}

  
void updateAngle() {
  sixDOF.getYawPitchRoll(angles);
  prevAngles[prevAngleI] = angles[1];
  prevAngleI = (prevAngleI + 1) % AvgAngles;
  float sum = 0;
  for (int i = 0; i < AvgAngles; i++)
      sum += prevAngles[i];
  currAngle = sum / AvgAngles;
  prevAngle = currAngle;
  
}
