#define ENC_FORWARD -1;
#define ENC_BACKWARD 1;

const int L_ENCODER_A = 24;
const int L_ENCODER_B = 25;
const int R_ENCODER_A = 52;
const int R_ENCODER_B = 53;

volatile long encoderRCount = 0;
volatile long encoderLCount = 0;

bool leftEncTimeState, rightEncTimeState = true;
long leftEnc_t1, rightEnc_t1 = 0;
long leftEnc_t2, rightEnc_t2 = 0;
long leftEnc_dt, rightEnc_dt = 0;

int leftEncDirection, rightEncDirection = ENC_FORWARD;

const int THRESHOLD_DT = 50; // number of milliseconds of not moving for speed to count as 0
const float MILLIS_TO_SEC = 1000;

// digital low pass filter (smooth function)
float smoothedRightSpeed = 0;

// MOTORS
//const int R_MOTOR_PWM = 6;
//const int R_MOTOR_IN_A = 4;
//const int R_MOTOR_IN_B = 5;

void setup() {
  Serial.begin(9600);
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);

  attachInterrupt(L_ENCODER_A, leftEncoder, RISING);
  attachInterrupt(R_ENCODER_A, rightEncoder, RISING);
  
//  digitalWrite(R_MOTOR_IN_A, 1);
//  digitalWrite(R_MOTOR_IN_B, 0);
}

void loop() {
//    analogWrite(R_MOTOR_PWM, 50);
//    smoothedRightSpeed = smooth(getRawRightSpeed(), 0.95, smoothedRightSpeed);
    Serial.print(getRawLeftSpeed()); Serial.print('\t'); Serial.println(getRawRightSpeed());
}

float getRawRightSpeed() {
  if ((millis() - rightEnc_t2) < THRESHOLD_DT) {
    return rightEncDirection * (MILLIS_TO_SEC / rightEnc_dt);
  } else {
    return 0;
  }
}

float getRawLeftSpeed() {
  if ((millis() - leftEnc_t2) < THRESHOLD_DT) {
    return leftEncDirection * (MILLIS_TO_SEC / leftEnc_dt);
  } else {
    return 0;
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

void leftEncoder() {
//  Serial.println("in the interrupt");
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

/*
  from http://playground.arduino.cc/Main/Smooth
  written by Paul Badger
  
  int sensVal - the sensor variable - raw material to be smoothed

  float  filterVal - The filter value is a float and must be between 0 and .9999 say. 0 is off (no smoothing) and .9999 is maximum smoothing.
    The actual performance of the filter is going to be dependent on fast you are sampling your sensor (the total loop time), so 
    some trial and error will probably be neccessary to get the desired response.

  smoothedVal - Use this for the output of the sensor and also feed it back into the loop. Each sensor needs its own value.
    Don't use this variable for any other purpose.
*/
int smooth(float data, float filterVal, float smoothedVal){
  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}
