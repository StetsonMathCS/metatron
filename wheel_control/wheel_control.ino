#include <Sabertooth.h>

Sabertooth ST(128);

void setup() {
  SabertoothTXPinSerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  while (Serial.available() > 0) {
    // first int is motor (1-4)
    // second int is speed (-127 - 127)
    int motor = Serial.parseInt();
    int speed = Serial.parseInt();
    if(motor >= 1 && motor <= 4 && speed >= -127 && speed <= 127) {
      ST.motor(motor, speed);
    } else {
      // bogus command, so kill motors
      for(int i = 1; i <= 4; i++) {
        ST.motor(i, 0);
      }
    }
  }
}
