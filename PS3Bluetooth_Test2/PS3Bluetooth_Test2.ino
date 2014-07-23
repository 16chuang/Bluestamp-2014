#include <PS3BT.h>
#include <usbhub.h>
#include <SPI.h>

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB usb;
BTD btd(&usb); // bluetooth dongle
PS3BT PS3(&btd); // PS3 controller bluetooth

const int JOYSTICK_DEADBAND = 28;

void setup() {
  // COPIED FROM EXAMPLE SKETCH
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void loop() {
  usb.Task();

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    Serial.println(mapJoystickValue(PS3.getAnalogHat(LeftHatY), 140.0));
  }
}

float mapJoystickValue(int joyValue, float outputMax) {
  if (joyValue >= 127.5 + JOYSTICK_DEADBAND) { // backwards
    return scale(joyValue, 150.0, 255, 0, -1.0 * outputMax);
  } else if (joyValue <= 127.5 - JOYSTICK_DEADBAND) { // forwards
    return scale(joyValue, 105.0, 0, 0, outputMax);
  } else { // don't move
    return 0.0;
  }
}

float scale(int input, float inputMin, int inputMax, int outputMin, float outputMax) {
  float output = 0;
  output = abs(input - inputMin) / abs(inputMax - inputMin) * (outputMax - outputMin);
  constrain(output, outputMin, outputMax);
  return int(output);
}
