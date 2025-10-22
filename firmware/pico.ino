#include <Adafruit_BNO055.h>

#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// Sensor objects
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

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

  if (!bno.begin())
  {
    Serial.print("BNO not detected");
    while (1);
  }
  if (! bmp.begin_I2C()) {  
    Serial.println("BMP not detected");
    while (1);
  }


  // Attach servos
  wristServo.attach(wristPin, 500, 2500);
  shoulderServo.attach(shoulderPin, 500, 2500);
  elbowServo.attach(elbowPin, 500, 2500);

  Serial.println("Servos ready");
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

  if (millis() % 200 == 0) { // every 200 ms (~5 Hz)
    sensors_event_t accelEvent;
    sensors_event_t tempEvent;
    sensors_event_t pressureEvent;

    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    float temperature = bmp.temperature;       // °C
    float pressure = bmp.pressure / 100.0;     // hPa
    float altitude = bmp.readAltitude(1013.25); // meters (assuming sea level pressure)

    // Send in simple CSV for ROS serial bridge
    Serial.print("ACC:");
    Serial.print(accelEvent.acceleration.x, 2); Serial.print(",");
    Serial.print(accelEvent.acceleration.y, 2); Serial.print(",");
    Serial.print(accelEvent.acceleration.z, 2);

    Serial.print(";BMP:");
    Serial.print(temperature, 2); Serial.print(",");
    Serial.print(pressure, 2); Serial.print(",");
    Serial.println(altitude, 2);
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

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}