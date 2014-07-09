// motor PWM 
const int MOTOR_PWM = 5; 

// change direction of motors
// A = 1, B = 1 -- brake to Vcc
// A = 1, B = 0 -- clockwise
// A = 0, B = 1 -- counterclockwise
// A = 0, B = 0 -- brake to GND
const int MOTOR_IN_A = 3;
const int MOTOR_IN_B = 4;

/* ENCODERS 
 * ----------------------------- */

// encoder ports
const int MOTOR_ENCODER_B = 12;
const int MOTOR_ENCODER_A = 13;

// encoder reading
int encoderValue = 0;

// read function prototype
void readEncoder();

void setup() {
  Serial.begin(9600);
  
  // Arduino pins to motor driver
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN_A, OUTPUT);
  pinMode(MOTOR_IN_B, OUTPUT);
  pinMode(MOTOR_ENCODER_A, INPUT);
  pinMode(MOTOR_ENCODER_B, INPUT);
}

void loop() {
  // PWM values: 25% = 64; 50% = 127; 75% = 191; 100% = 255
  // control speed
  analogWrite(MOTOR_PWM, 127);

  // turn counter-clockwise
  digitalWrite(MOTOR_IN_B, LOW);
  digitalWrite(MOTOR_IN_B, HIGH);
  
//  readEncoder();

  delay(10);
}

int currentReadingA = LOW;
int currentReadingB = LOW;
int previousReadingA = LOW;

void readEncoder() {
  // get current encoder readings
  currentReadingA = digitalRead(MOTOR_ENCODER_A);
  currentReadingB = digitalRead(MOTOR_ENCODER_B);

  if (previousReadingA == LOW && currentReadingA == HIGH) { // coming up on an edge
    if (currentReadingB == HIGH) {
    	encoderValue++; // clockwise
    } else {
    	encoderValue--; // counterclockwise
    }
  }

  Serial.println(encoderValue);

  previousReadingA = currentReadingA;
}
