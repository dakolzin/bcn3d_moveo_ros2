const int fsrPin1 = A1; 
const int fsrPin2 = A2; 

void setup() {
  Serial.begin(115200);
}

void loop() {
  int fsrReading1 = analogRead(fsrPin1);
  int fsrReading2 = analogRead(fsrPin2);

  Serial.print(fsrReading1);
  Serial.print(",");
  Serial.println(fsrReading2);

  delay(50); // Небольшая задержка для стабилизации данных
}
