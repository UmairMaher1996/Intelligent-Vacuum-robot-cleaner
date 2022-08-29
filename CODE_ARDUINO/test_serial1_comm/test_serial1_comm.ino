void setup() {
  // put your setup code here, to run once:

  Serial2.begin(115200);
  Serial1.begin(115200);
  Serial.begin(115200);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial2.println("Serial 2");
  Serial1.println("Serial 1");
  Serial.println("Serial Standard");
  delay(10);
}
