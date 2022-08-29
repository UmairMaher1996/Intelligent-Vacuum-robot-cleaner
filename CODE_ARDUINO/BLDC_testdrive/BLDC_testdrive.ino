#include <Servo.h>

Servo motor1;

int val;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motor1.attach(3,1000,3000);
}

void loop() {
  // put your main code here, to run repeatedly:

  val = analogRead(A0);
  val = map(val,0,1023,0,256);
  motor1.write(val);
  Serial.println(val);
  delay(100); 

}
