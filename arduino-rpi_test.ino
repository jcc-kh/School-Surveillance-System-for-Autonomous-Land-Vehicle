/*void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("Hello from Arduino!");
  delay(1000);
}*/
void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    int data = Serial.readStringUntil('\n');
    Serial.print("You sent me: ");
    Serial.println(data);
  }
}
