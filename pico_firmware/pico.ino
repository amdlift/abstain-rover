// This code is a PLACEHOLDER

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "LED_ON") {
      digitalWrite(25, HIGH);
      Serial.println("ACK: LED ON");
    } else if (cmd == "LED_OFF") {
      digitalWrite(25, LOW);
      Serial.println("ACK: LED OFF");
    }
  }

  // Periodically send sensor data
  int sensor = analogRead(A0);
  Serial.print("SENSOR:");
  Serial.println(sensor);
  delay(100);
}
