
#include <MSMotorShield.h>

MS_DCMotor **motors;

void setup() {
  motors = (MS_DCMotor**)malloc(sizeof(MS_DCMotor*)*4);
  for(int i = 0; i < 4; i++) {
    motors[i] = new MS_DCMotor(i+1);
    motors[i]->run(RELEASE);
  }
  Serial.begin(9600);
}

void loop() {
  while (Serial.available() > 0) {
    // parse type of command: M = motor, D = delay
    char command = Serial.read();
    if(command == 'M') {
      // read motor number (1-4)
      int m = Serial.parseInt() - 1; // subtract 1 to get motors array index
      if(m < 0 || m > 3) { // bad command
        for(int i = 0; i < 4; i++) {
          motors[i]->run(RELEASE);
        }
        continue; 
      }
      // read forwards (F) vs. backwards (B) vs. release (R)
      char dir = Serial.read();
      if(dir == 'F') {
        motors[m]->run(FORWARD);
        // now get speed
        int speed = Serial.parseInt();
        motors[m]->setSpeed(speed);
      } else if(dir == 'B') {
        motors[m]->run(BACKWARD);
        // now get speed
        int speed = Serial.parseInt();
        motors[m]->setSpeed(speed);
      } else if(dir == 'R') {
        motors[m]->run(RELEASE);
      } else { // bad command
        for(int i = 0; i < 4; i++) {
          motors[i]->run(RELEASE);
        }
        continue;
      }
    } else if(command == 'D') {
      // read how much to delay (milliseconds)
      int d = Serial.parseInt();
      delay(d);
    }
  }
}
