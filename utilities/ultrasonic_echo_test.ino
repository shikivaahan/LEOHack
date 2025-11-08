/*
  Ultrasonic echo test

  This sketch pulses TRIG and actively polls ECHO for up to 30 ms.
  It prints one of:
    ECHO_SEEN <elapsed_us> <pulseIn_us>
    NO_ECHO

  This helps diagnose whether the ECHO pin ever goes HIGH after triggering.

  Wiring (default pins):
    TRIG -> D9
    ECHO -> D10
    VCC -> 5V
    GND -> GND
*/

const int trigPin = 9;
const int echoPin = 10;
const int ledPin = LED_BUILTIN;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  delay(200);
  Serial.println("ULTRASONIC ECHO TEST START");
}

void loop() {
  // clear trig
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // send trigger
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long start = micros();
  bool seen = false;
  unsigned long seenAt = 0;

  // poll ECHO for up to 30000 us
  while (micros() - start < 30000UL) {
    if (digitalRead(echoPin) == HIGH) {
      seen = true;
      seenAt = micros() - start;
      break;
    }
  }

  unsigned long pi = pulseIn(echoPin, HIGH, 30000UL);

  if (seen) {
    Serial.print("ECHO_SEEN ");
    Serial.print(seenAt);
    Serial.print("us pulseIn=");
    Serial.println(pi);
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
  } else {
    Serial.println("NO_ECHO");
  }

  delay(200);
}
