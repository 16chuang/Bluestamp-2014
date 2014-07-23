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
    Serial.print("L X"); Serial.print(PS3.getAnalogHat(LeftHatX)); Serial.print('\t');
    Serial.print("R Y"); Serial.print(PS3.getAnalogHat(RightHatY)); Serial.println('\t');
  }
}
