#include <Adafruit_BNO055.h>

#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// Sensor objects
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

// Set UART pins for OpenLog
const int dataRX = 5;
const int dataTX = 4;

// Servo objects
Servo wristServo;
Servo shoulderServo;
Servo elbowServo;

// Servo pins
const int wristPin = 16;
const int shoulderPin = 17;
const int elbowPin = 18;

// DC Motor pins
const int AIN1 = 11;
const int AIN2 = 10;
const int PWMA = 9;
const int BIN1 = 13;
const int BIN2 = 14;
const int PWMB = 15;

// Buffer for incoming serial data
String inputString = "";
bool stringComplete = false;


void setup() {
  Serial.begin(115200); // Use Serial for RP2040 USB UART, or Serial for default
  Serial2.setRX(dataRX);
  Serial2.setTX(dataTX);
  Serial2.begin(9600);
  inputString.reserve(100);

  Wire1.setSCL(3);
  Wire1.setSDA(2);

  if (!bno.begin())
  {
    Serial.print("BNO not detected");

  }
  if (! bmp.begin_I2C(0x77, &Wire1)) {  
    Serial.println("BMP not detected");
  }


  // Attach servos
  wristServo.attach(wristPin, 500, 2500);
  shoulderServo.attach(shoulderPin, 500, 2500);
  elbowServo.attach(elbowPin, 500, 2500);

  Serial.println("Servos ready");

  // Motor setup
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  // stopMotors();

  Serial.println("Motors ready");
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
    parseAndMoveServosMotors(inputString);
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

    Serial2.print("ACC:");
    Serial2.print(accelEvent.acceleration.x, 2); Serial2.print(",");
    Serial2.print(accelEvent.acceleration.y, 2); Serial2.print(",");
    Serial2.print(accelEvent.acceleration.z, 2);

    Serial2.print(";BMP:");
    Serial2.print(temperature, 2); Serial2.print(",");
    Serial2.print(pressure, 2); Serial2.print(",");
    Serial2.println(altitude, 2);
  }

}

// ====== Motor Control Helpers ======
void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void setMotor(int in1, int in2, int pwm, float speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}

// ====== Command Parser ======
void parseAndMoveServosMotors(String data) {
  // Example: "W:90,S:45,E:120,A:100,C:-80"
  int wIndex = data.indexOf("W:");
  int sIndex = data.indexOf("S:");
  int eIndex = data.indexOf("E:");
  int aIndex = data.indexOf("A:");
  int cIndex = data.indexOf("C:");

  float wAngle = (wIndex != -1) ? data.substring(wIndex + 2, nextComma(data, wIndex)).toFloat() : -1;
  float sAngle = (sIndex != -1) ? data.substring(sIndex + 2, nextComma(data, sIndex)).toFloat() : -1;
  float eAngle = (eIndex != -1) ? data.substring(eIndex + 2, nextComma(data, eIndex)).toFloat() : -1;
  float aSpeed = (aIndex != -1) ? data.substring(aIndex + 2, nextComma(data, aIndex)).toFloat() : 0;
  float cSpeed = (cIndex != -1) ? data.substring(cIndex + 2).toFloat() : 0;

  if (wAngle >= 0) wristServo.write(constrain(wAngle, 0, 180));
  if (sAngle >= 0) shoulderServo.write(constrain(sAngle, 0, 180));
  if (eAngle >= 0) elbowServo.write(constrain(eAngle, 0, 180));

  setMotor(AIN1, AIN2, PWMA, aSpeed);
  setMotor(BIN1, BIN2, PWMB, cSpeed);

  Serial.print("Cmd -> W:"); Serial.print(wAngle);
  Serial.print(" S:"); Serial.print(sAngle);
  Serial.print(" E:"); Serial.print(eAngle);
  Serial.print(" A:"); Serial.print(aSpeed);
  Serial.print(" C:"); Serial.println(cSpeed);
}

int nextComma(String s, int start) {
  int idx = s.indexOf(",", start);
  return (idx == -1) ? s.length() : idx;
}