#include<PID_v1.h> 

#define RIGHT_ENCA 2 // Yellow wire
#define RIGHT_ENCB 7 // White wire
#define RPWM 5
#define LPWM 3
#define EN_MOTOR_RIGHT 8

double left_kp = 1 , left_ki = 0 , left_kd = 0.01;            
double right_kp = 4 , right_ki = 0 , right_kd = 0.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

float demandx=0;
float demandz=0;

double demand_speed_left;
double demand_speed_right;

unsigned long currentMillis;
unsigned long prevMillis;

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;



void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(RIGHT_ENCA, INPUT);
  pinMode(RIGHT_ENCB, INPUT);
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(EN_MOTOR_RIGHT,OUTPUT);
  digitalWrite(EN_MOTOR_RIGHT, LOW);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA),readEncoder_Right,RISING);
  Startmotor_Right(0,0);
}

void loop() {
  
 // set target position
  int target = 700;
  //int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0.01;
  float ki = 0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

 // Read the position
  pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  if(((pos-target) <= 5) && (pos>target)){
    pwr = 0;
    }

  // motor direction
  int dir = -1;
  if(u<0){
    dir = 1;
  }

  // signal the motor
  Startmotor_Right(dir,pwr);


  // store previous error
  eprev = e;
  
  Serial.print(pwr);
  Serial.print(" ");
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();




}

void Startmotor_Right(int dir, int pwm){
  digitalWrite(EN_MOTOR_RIGHT, HIGH);
  if(dir == 1){
    analogWrite(LPWM,0);
    analogWrite(RPWM,pwm);
    }
  else if(dir == -1){
    analogWrite(LPWM,pwm);
    analogWrite(RPWM,0);
    }
  else{
  analogWrite(LPWM,0);
  analogWrite(RPWM,0);
  digitalWrite(EN_MOTOR_RIGHT, LOW);
  }
}

void readEncoder_Right(){
  int b = digitalRead(RIGHT_ENCB);
  if(b>0){
    posi++;
    }
  else{
    posi--;
  }
}  
