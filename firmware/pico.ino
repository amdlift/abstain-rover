#include <Arduino.h>
#include <Servo.h>

// Servo objects
Servo wristServo;
Servo shoulderServo;
Servo elbowServo;

// Servo pins
const int wristPin = 16;
const int shoulderPin = 17;
const int elbowPin = 18;

// Buffer for incoming serial data
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200); // Use Serial for RP2040 USB UART, or Serial for default
  inputString.reserve(100);

  // Attach servos
  wristServo.attach(wristPin, 500, 2500);
  shoulderServo.attach(shoulderPin, 500, 2500);
  elbowServo.attach(elbowPin, 500, 2500);

  Serial.println("Pico ready");
}

void loop() {
  // Read serial input
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  // If a full line received, parse and update servos
  if (stringComplete) {
    parseAndMoveServos(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void parseAndMoveServos(String data) {
  // Expected format: "B:90,S:45,E:120"
  int wIndex = data.indexOf("W:");
  int sIndex = data.indexOf("S:");
  int eIndex = data.indexOf("E:");

  if (wIndex == -1 || sIndex == -1 || eIndex == -1) return;

  float wAngle = data.substring(wIndex + 2, sIndex - 1).toFloat();
  float sAngle = data.substring(sIndex + 2, eIndex - 1).toFloat();
  float eAngle = data.substring(eIndex + 2).toFloat();

  // Constrain angles to 0-180
  wAngle = constrain(wAngle, 0, 180);
  sAngle = constrain(sAngle, 0, 180);
  eAngle = constrain(eAngle, 0, 180);

  // Move servos
  wristServo.write(wAngle);
  shoulderServo.write(sAngle);
  elbowServo.write(eAngle);

  // Optional debug
  Serial.print("Moved servos: ");
  Serial.print(wAngle); Serial.print(",");
  Serial.print(sAngle); Serial.print(",");
  Serial.println(eAngle);
}
