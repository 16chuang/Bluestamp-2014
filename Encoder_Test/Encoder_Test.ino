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

void leftEncoder() {
  if (digitalRead(L_ENCODER_B) == LOW) {
    encoderLCount++;
    leftEncDirection = ENC_FORWARD;
  } else {
    encoderLCount--;
    leftEncDirection = ENC_BACKWARD;
  }
  
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
  if (digitalRead(R_ENCODER_B) == LOW) {
    encoderRCount--;
    rightEncDirection = ENC_FORWARD;
  } else {
    encoderRCount++;
    rightEncDirection = ENC_BACKWARD;
  }
  
  if (rightEncTimeState) {
    rightEnc_t1 = millis();
    rightEncTimeState = false;
  } else {
    rightEnc_t2 = millis();
    rightEnc_dt = rightEnc_t1 - rightEnc_t2;
    rightEncTimeState = true;
  }
}
