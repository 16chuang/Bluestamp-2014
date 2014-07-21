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
#include <CommunicationUtils.h>
#include <DebugUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
#include <stdio.h>

FreeSixIMU sixDOF = FreeSixIMU();

void setup() {
  // Serial Connection
  Serial.begin(9600);
  // Wait a while so that slow humans can open the serial console
  delay(2000);

  // Initialize the TWI peripheral and randomly wait a bit
  Serial.print("Initializing TWI peripheral... ");
  Wire.begin();
  delay(5);
  Serial.print("Done!\n");
  
  // Initialize our instance of the sixDOF library and randomly a bit more
  Serial.print("Initializing sixDOF... ");
  sixDOF.init();
  delay(5);
}


void loop() {
  /*Wire.beginTransmission(0x53);  // Set up a write to the accelerometer
  Wire.write(0x32);  // Send the register address
  Wire.endTransmission();

  Wire.requestFrom(0x53, 6);
  for(int a = 0; a < 6; a++)
    Wire.read();*/
  
  int16_t rawAcc[] = {0, 1, 2};
  float rawGyro[] = {0, 0, 0};
  sixDOF.acc.readAccel(&rawAcc[0], &rawAcc[1], &rawAcc[2]);
  sixDOF.gyro.readGyro(&rawGyro[0], &rawGyro[1], &rawGyro[2]);
  
//  char str[256];
//  snprintf(str, sizeof(str), "a[y]: %4.3f\tg[x]: %4.0d\n", raw[1] * 0.004f, raw[3]);
//  
//  Serial.print(str);

  Serial.print(rawAcc[1]); Serial.print('\t'); Serial.println(rawGyro[0]);

//  Serial.print(raw[1]); Serial.print('\t'); Serial.println(raw[3]);
  
//  delay(250);
}
