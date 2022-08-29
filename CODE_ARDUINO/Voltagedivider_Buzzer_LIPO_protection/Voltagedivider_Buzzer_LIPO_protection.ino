const int BatteryPin = A0;
const int Buzzer = 10;
float LIPO_volt;


void setup() {
  Serial.begin(9600);
  pinMode(Buzzer, OUTPUT);

}

void loop() {
    LIPO_volt = (((analogRead(BatteryPin) * 5.0 / 1023.0)/1.2) * (1.2+6.2)); //voltagedivider 1.2k+6.2k resistors
    if(LIPO_volt < 9.6){
      tone(Buzzer, 1000);
      Serial.println("LOW BATTERY");
      }
    else{noTone(Buzzer);}  
    Serial.print("LIPO_Volt:");
    Serial.println(LIPO_volt);
    delay(100);
}
