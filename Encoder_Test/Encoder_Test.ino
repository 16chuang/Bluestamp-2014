#define ENC_FORWARD 1;
#define ENC_BACKWARD -1;

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

// digital low pass filter (smooth function)
float smoothedRightSpeed = 0;

void setup() {
  Serial.begin(9600);
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);

  attachInterrupt(0, leftEncoder, RISING); // pin 2, low to high
  attachInterrupt(1, rightEncoder, RISING); // pin 3, low to high
}

void loop() {
  smoothedRightSpeed = smooth(getRawRightSpeed(), 0.9, smoothedRightSpeed);
  Serial.println(smoothedRightSpeed);
}

float getRawLeftSpeed() {
  if ((millis() - leftEnc_t2) < THRESHOLD_DT) {
    return leftEncDirection * (MILLIS_TO_SEC / leftEnc_dt);
  } else {
    return 0;
  }
}

float getRawRightSpeed() {
  if ((millis() - rightEnc_t2) < THRESHOLD_DT) {
    return rightEncDirection * (MILLIS_TO_SEC / rightEnc_dt);
  } else {
    return 0;
  }
}

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
