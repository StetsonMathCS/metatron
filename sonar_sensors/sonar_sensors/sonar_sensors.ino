
#define ECHO_PIN 2 // D2
#define TRIG_PIN 3 // D3
long duration = -1;
long distance = -1;

void setup() {
  // for debugging
  Serial.begin(9600);
  Serial.println("starting...");
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {

  // technique adopted from www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // duration is in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // divide duration by 2 to get duration one-way
  // then divide by 29.1 to get centimeters (29.1545 microseconds/cm is speed of sound)
  distance = (duration/2) / 29.1545;

  Serial.print("Duration: ");
  Serial.println(duration);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm");
  Serial.println();
  delay(1000);
}
