#include <Sabertooth.h>

Sabertooth ST(128);
Sabertooth ST2(129);
int motor[4];
int speed[4];
int i;
int ok;

void setup() {
  SabertoothTXPinSerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  while (Serial.available() > 0) {
    // always get four pairs of numbers (each pair is motor, speed)

    for(i = 0; i < 4; i++) {
      // first int is motor (1-4)
      // second int is speed (-127 - 127)
      motor[i] = Serial.parseInt();
      speed[i] = Serial.parseInt();
    }
    // check validity of command
    ok = 1;
    for(i = 0; i < 4; i++) {
      if(motor[i] < 1 || motor[i] > 4 || speed[i] < -127 || speed[i] > 127) {
        ok = 0;
        break;
      }
    }
    if(ok == 1) {
      for(i = 0; i < 4; i++) {
        if(motor[i] >= 1 && motor[i] <= 2) {
          ST.motor(motor[i], speed[i]);
        } else {
          ST2.motor(motor[i]-2, speed[i]);
        }
      }
    } else {
      // bogus command, so kill motors
      for(int i = 1; i <= 2; i++) {
        ST.motor(i, 0);
      }
      for(int i = 1; i <= 2; i++) {
        ST2.motor(i, 0);
      }
    }
  }
}

