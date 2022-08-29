//1m = 5850 encoder counts (still to reconfirm)

#include <PID_v1.h>

//PID Values for Right Wheel
double Pk0 = 1;
double Ik0 = 0;
double Dk0 = 0.02;

double Setpoint0, Input0, Output0, Output0a;
PID PID0(&Input0, &Output0, &Setpoint0, Pk0, Ik0, Dk0, DIRECT);

//PID Values for Let Wheel
double Pk1 = 1;
double Ik1 = 0;
double Dk1 = 0.02;

double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);


float demand0;
float demand1;

float demandx;
float demandz;

unsigned long currentMillis;
unsigned long previousMillis;


////RIGHT MOTOR
#define encoder0PinA 2 //Encoder 0 (Right Wheel) // Yellow wire
#define encoder0PinB 3 // White wire

#define Right_RPWM 4 //Right Motor PWM-pins
#define Right_LPWM 5
#define EN_MOTOR_RIGHT 47 //Rght Motor Enable pin


////LEFT MOTOR
#define encoder1PinA 21 //Encoder 1 (Left Wheel) // Yellow wire
#define encoder1PinB 20 // White wire

#define Left_RPWM 9 //Left Motor PWM-pins
#define Left_LPWM 8
#define EN_MOTOR_LEFT 49 //Left Motor Enable pin


volatile long encoder0Pos = 0; //encoder Right
volatile long encoder1Pos = 0; //encoder Left

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

int dir_0;
float pwr_0;


void setup() {
  
  //Right Wheel
  pinMode(Right_RPWM, OUTPUT);
  pinMode(Right_LPWM, OUTPUT);
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA),doEncoderA_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB),doEncoderB_right, CHANGE);
  pinMode(EN_MOTOR_RIGHT,OUTPUT);
  digitalWrite(EN_MOTOR_RIGHT, LOW);

  //Left Wheel
  pinMode(Left_RPWM, OUTPUT);
  pinMode(Left_LPWM, OUTPUT);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA),doEncoderA_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB),doEncoderB_left, CHANGE);
  pinMode(EN_MOTOR_LEFT,OUTPUT);
  digitalWrite(EN_MOTOR_LEFT, LOW);

  //PID_Right
  PID0.SetMode(AUTOMATIC);
  PID0.SetOutputLimits(-255,255); //Limit maximum PWM
  PID0.SetSampleTime(10);

  //PID_Left
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255,255);
  PID1.SetSampleTime(10);
  
  //Serial monitor
  Serial.begin(115200);
}

void loop() {

  currentMillis= millis();
  if(currentMillis - previousMillis >= 10){ //start of timed loop = 10ms cycle
    previousMillis = currentMillis;

    if(Serial.available()>0){
      char c = Serial.read();

      if(c == 'a'){ //drive at 0.5 m/s

        demandx = 0.5;
        demandz = 0;
        
      }
      if(c == 's'){ //drive at 0.5 m/s

        demandx = -0.5;
        demandz = 0;
        
      }
      else if(c == 'f'){//drive at 1 m/s

        demandx = 1;
        demandz = 0;
        
      }
      else if(c == 'b'){//drive at 0.25 m/s

        demandx = 0.25;
        demandz = 0;
        
      }
      else if(c == 'c'){//drive at 0.25 m/s + turn at 1 rad/s

        demandx = 0.25;
        demandz = 1;
        
      }
      else if(c == 'd'){//turn at 1 rad/second CW

        demandx = 0;
        demandz = 1;
        
      }
      else if(c == 'e'){//turn at 1 rad/second CCW

        demandx = 0;
        demandz = -1;
        
      }
      else if(c == 'q'){//STOP

        demandx = 0;
        demandz = 0;

      }
    }

    /*  5650 encodercount for 1m. 56.5 encodercounts met 10 millisecond loop at 1 m/s velocity
     *  
     *  Distance between wheels is 320mm. Half = 160mm
     *  Circumfrence of 320mm circle is: 320*pi = 1005mm, to turn 180 degrees, each wheel needs to drive half of 1005mm, or 502mm 
     *  So to turn 180 degrees, each wheel needs to trun 502/pi = 160mm
     *  
     */

    demand0 = demandx - (demandz*0.160);//differential drive, ROS gives x and z variables. x = forward/backward, z = rotating
    demand1 = demandx + (demandz*0.160);

    
    encoder0Diff = encoder0Pos - encoder0Prev; //workout the difference from last time, this is the current speed in count per 10ms
    encoder1Diff = encoder1Pos - encoder1Prev;

    encoder0Error = (demand0*56.5) - encoder0Diff;
    encoder1Error = (demand1*56.5) - encoder1Diff;

    encoder0Prev = encoder0Pos;
    encoder1Prev = encoder1Pos;
    
    Serial.print("Right Position: ");
    Serial.print(encoder0Pos);
    Serial.print(" , ");
    Serial.print("Left Position: ");
    Serial.print(encoder1Pos);
    Serial.println("");

    //PID for Both Wheels
    Setpoint0 = demand0*56.5*5; //added factor 5 to match speed 1 m/s IRL >> calibration
    Setpoint1 = demand1*56.5*5;
    Input0 = encoder0Diff;
    Input1 = encoder1Diff;
    PID0.Compute();
    PID1.Compute();

/*
    //PID for Left Wheel
    Setpoint1 = demand1;
    Input1 = encoder1Pos;
    PID1.Compute();
*/
    if(Output0 > 0 ){
      //digitalWrite(EN_MOTOR_RIGHT, HIGH);
      Output0a = abs(Output0);
      analogWrite(Right_LPWM,Output0a);
      analogWrite(Right_RPWM,0);
     }

     else if(Output0 < 0){
      //digitalWrite(EN_MOTOR_RIGHT, HIGH);
      Output0a = abs(Output0);
      analogWrite(Right_LPWM,0);
      analogWrite(Right_RPWM,Output0a);

     }
     else {
      digitalWrite(EN_MOTOR_LEFT, LOW);
      analogWrite(Left_LPWM,0);
      analogWrite(Left_RPWM,0);
      }

     if(Output1 > 0){
      //digitalWrite(EN_MOTOR_LEFT, HIGH);
      Output1a = abs(Output1);
      analogWrite(Left_LPWM,0);
      analogWrite(Left_RPWM,Output1a);
     }

     else if(Output1 < 0){
      //digitalWrite(EN_MOTOR_LEFT, HIGH);
      Output1a = abs(Output1);
      analogWrite(Left_LPWM,Output1a);
      analogWrite(Left_RPWM,0);

     }
     else {
      //digitalWrite(EN_MOTOR_LEFT, LOW);
      analogWrite(Left_LPWM,0);
      analogWrite(Left_RPWM,0);
      }

      /*
      Serial.println("Output Right ");
      Serial.println(Output0);
      Serial.println("Output Left ");
      Serial.println(Output1);
      */
      
      digitalWrite(EN_MOTOR_LEFT, HIGH);
      digitalWrite(EN_MOTOR_RIGHT, HIGH);
      
      
    }// end of timed loop
}// end of main loop

// ****encoder interrupts****
//Encoder 0 (Right Wheel)

void doEncoderA_right(){

  if(digitalRead(encoder0PinA) == HIGH){
    if(digitalRead(encoder0PinB) == LOW){
      encoder0Pos = encoder0Pos - 1;
     }
     else{
      encoder0Pos = encoder0Pos + 1;
      }
    }
    else{
      if(digitalRead(encoder0PinB) == HIGH){
        encoder0Pos = encoder0Pos - 1;
        }
        else{
          encoder0Pos = encoder0Pos + 1;
        }
     }
  }

  void doEncoderB_right(){

  if(digitalRead(encoder0PinB) == HIGH){
    if(digitalRead(encoder0PinA) == LOW){
      encoder0Pos = encoder0Pos - 1;
     }
     else{
      encoder0Pos = encoder0Pos + 1;
      }
    }
    else{
      if(digitalRead(encoder0PinA) == LOW){
        encoder0Pos = encoder0Pos - 1;
        }
        else{
          encoder0Pos = encoder0Pos + 1;
        }
     }
  }

  //Encoder 1 (Left Wheel)

void doEncoderA_left(){

  if(digitalRead(encoder1PinA) == HIGH){
    if(digitalRead(encoder1PinB) == LOW){
      encoder1Pos = encoder1Pos + 1;
     }
     else{
      encoder1Pos = encoder1Pos - 1;
      }
    }
    else{
      if(digitalRead(encoder1PinB) == HIGH){
        encoder1Pos = encoder1Pos + 1;
        }
        else{
          encoder1Pos = encoder1Pos - 1;
        }
     }
  }

  void doEncoderB_left(){

  if(digitalRead(encoder1PinB) == HIGH){
    if(digitalRead(encoder1PinA) == LOW){
      encoder1Pos = encoder1Pos + 1;
     }
     else{
      encoder1Pos = encoder1Pos - 1;
      }
    }
    else{
      if(digitalRead(encoder1PinA) == LOW){
        encoder1Pos = encoder1Pos + 1;
        }
        else{
          encoder1Pos = encoder1Pos - 1;
        }
     }
  }
