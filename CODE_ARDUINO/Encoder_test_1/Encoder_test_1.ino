
#define ENCA 2 // Yellow wire
#define ENCB 3 // White wire
int pos = 0;

void setup() {

  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  Serial.println(pos);
  delay(100);
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
    }
  else{
    pos--;
  }
}
