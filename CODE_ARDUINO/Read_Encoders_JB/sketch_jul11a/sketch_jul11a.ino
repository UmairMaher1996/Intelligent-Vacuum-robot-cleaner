//1m = 5850 encoder counts (still to reconfirm)

unsigned long currentMillis;
unsigned long previousMillis;

#define encoder0PinA 2 //Encoder 1 (Right Wheel)
#define encoder0PinB 7

#define Right_RPWM 5 //Right Motor PWM-pins
#define Right_LPWM 3

volatile long encoder0Pos = 0;


void setup() {
  
  //Right Wheel
  pinMode(Right_RPWM, OUTPUT);
  pinMode(Right_LPWM, OUTPUT);
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(0,doEncoderA, CHANGE);
  attachInterrupt(1,doEncoderB, CHANGE);

  //Left Wheel

  
  //Serial monitor
  Serial.begin(115200);
}

void loop() {

  currentMillis= millis();

  if(currentMillis - previousMillis >= 10){ //start of timed loop
    previousMillis = currentMillis;

    Serial.print(encoder0Pos);
    Serial.print(" ");
    //Serial.println(encoder1Pos);
    
    }// end of timed loop
}// end of main loop

// ****encoder interrupts****
//Encoder 1

void doEncoderA(){

  if(digitalRead(encoder0PinA) == HIGH){
    if(digitalRead(encoder0PinB) == LOW){
      encoder0Pos = encoder0Pos + 1;
     }
     else{
      encoder0Pos = encoder0Pos - 1;
      }
    }
    else{
      if(digitalRead(encoder0PinB) == HIGH){
        encoder0Pos = encoder0Pos + 1;
        }
        else{
          encoder0Pos = encoder0Pos - 1;
        }
     }
  }

  void doEncoderB(){

  if(digitalRead(encoder0PinB) == HIGH){
    if(digitalRead(encoder0PinA) == LOW){
      encoder0Pos = encoder0Pos + 1;
     }
     else{
      encoder0Pos = encoder0Pos - 1;
      }
    }
    else{
      if(digitalRead(encoder0PinA) == LOW){
        encoder0Pos = encoder0Pos + 1;
        }
        else{
          encoder0Pos = encoder0Pos - 1;
        }
     }
  }

  //SAME FOR ENCODER LEFT WHEEL (OMGEKEERD NA TEST??)
