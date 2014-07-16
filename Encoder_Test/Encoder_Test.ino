const int L_ENCODER_A = 2;
const int L_ENCODER_B = 7;
const int R_ENCODER_A = 3;
const int R_ENCODER_B = 8;

volatile long encoderRCount = 0;
volatile long encoderLCount = 0;

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
  Serial.print(encoderLCount); Serial.print('\t'); Serial.println(encoderRCount);
}

void leftEncoder() {
  if (digitalRead(L_ENCODER_B) == LOW) {
    encoderLCount++;
  } else {
    encoderLCount--;
  }
}

void rightEncoder() {
  if (digitalRead(R_ENCODER_B) == LOW) {
    encoderRCount++;
  } else {
    encoderRCount--;
  }
}
