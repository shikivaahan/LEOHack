/*
  Ultrasonic Sensor HC-SR04 and Arduino
  (Formatted and added to repo)

  Wiring (example):
    VCC  -> 5V
    GND  -> GND
    TRIG -> digital pin 9
    ECHO -> digital pin 10

  This sketch measures distance using the HC-SR04 and prints the result to Serial.
  Open the Arduino IDE, select the correct board/port, open this file and upload.
*/

const int trigPin = 12;
const int echoPin = 11;

long duration;
int distance;

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  Serial.begin(9600);       // Starts the serial communication
}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2; // speed of sound = 0.034 cm/us

  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(200); // small delay between readings
}
