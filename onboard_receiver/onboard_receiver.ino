#include <SPI.h>
#include <WiFiNINA.h>
#include <Servo.h>
#include "motor_control.h"
#include "student_functions.h"

// ===== Wi-Fi setup =====
char ssid[] = "Nano_12345678_AP";
char pass[] = "nano1pwd";

int status = WL_IDLE_STATUS;
WiFiServer server(8080);  // TCP server on port 8080

// ===== Ultrasonic setup =====
const int trigPin = 12;
const int echoPin = 11;
long duration;
int distance;

// ===== Function to read ultrasonic distance =====
int readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000); // timeout to avoid lockup
  if (duration == 0) return -1; // no echo received
  distance = duration * 0.034 / 2; // speed of sound in cm/us
  return distance;
}

// ===== Setup =====
void setup() {
  Serial.begin(9600);

  // --- Ultrasonic pins ---
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Creating an access point...");
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Failed to start AP");
  }

  delay(5000); // allow AP to initialize

  IPAddress ip = WiFi.localIP();
  Serial.print("AP IP Address: ");
  Serial.println(ip);

  server.begin();

  // initialize servos and motors
  servo_init();
  motor_init();
}

// ===== Main loop =====
void loop() {
  WiFiClient client = server.available(); // Listen for incoming clients

  if (client) {
    Serial.println("New client connected.");
    unsigned long lastSend = millis();

    while (client.connected()) {
      // --- Handle incoming control commands ---
      while (client.available()) {
        char c = client.read();
        Serial.write(c);

        if (c == 's') {
          stop();
          Serial.println("Motors Stopped");
        }
        else if (c == 'f') {
          char _ = client.read(); // skip space
          int power = client.read() - '0';
          _ = client.read();
          int time = client.read() - '0';
          go_forward(power, time);
        }
        else if (c == 'b') {
          char _ = client.read();
          int power = client.read() - '0';
          _ = client.read();
          int time = client.read() - '0';
          go_backward(power, time);
        }
        else if (c == 'l') {
          char _ = client.read();
          int power = client.read() - '0';
          _ = client.read();
          int time = client.read() - '0';
          turn_left(power, time);
        }
        else if (c == 'r') {
          char _ = client.read();
          int power = client.read() - '0';
          _ = client.read();
          int time = client.read() - '0';
          turn_right(power, time);
        }
        else if (c == 'a') {
          char _ = client.read();
          int power = client.read() - '0';
          _ = client.read();
          int time = client.read() - '0';
          translate_left(power, time);
        }
        else if (c == 'd') {
          char _ = client.read();
          int power = client.read() - '0';
          _ = client.read();
          int time = client.read() - '0';
          translate_right(power, time);
        }
        else if (c == 'o') {
          servo_open();
          Serial.println("O function triggered");
        }
        else if (c == 'p') {
          servo_close();
          Serial.println("P function triggered");
        }

        // Echo command back for acknowledgement
        client.write(c);
      }

      // --- Send distance data periodically ---
      if (millis() - lastSend >= 200) { // every 200ms
        int dist = readDistance();
        Serial.print("Distance: ");
        if (dist == -1) Serial.println("No echo");
        else Serial.print(dist), Serial.println(" cm");

        // Send distance to client
        client.print("DIST:");
        if (dist == -1) client.println("N/A");
        else client.println(dist);

        lastSend = millis();
      }
    }

    client.stop();
    Serial.println("Client disconnected.");
  }
}
