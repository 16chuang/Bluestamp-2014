// motor PWM 
const int MOTOR_PWM = 5; 

// change direction of motors
// A = 1, B = 1 -- brake to Vcc
// A = 1, B = 0 -- clockwise
// A = 0, B = 1 -- counterclockwise
// A = 0, B = 0 -- brake to GND
const int MOTOR_IN_A = 6;
const int MOTOR_IN_B = 7;

void setup() {
  Serial.begin(9600);
  
  // Arduino pins to motor driver
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN_A, OUTPUT);
  pinMode(MOTOR_IN_B, OUTPUT);
}

void loop() {
  // PWM values: 25% = 64; 50% = 127; 75% = 191; 100% = 255
  // control speed
  analogWrite(MOTOR_PWM, 127);

  // turn counter-clockwise
  digitalWrite(MOTOR_IN_A, LOW);
  digitalWrite(MOTOR_IN_B, HIGH);

  delay(10);
}
