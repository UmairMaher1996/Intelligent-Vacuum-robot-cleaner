////RIGHT MOTOR
#define ENCA 2 // Yellow wire
#define ENCB 3 // White wire
#define RPWM 4
#define LPWM 5
#define EN_MOTOR1 47

/*
/////LEFT MOTOR
#define ENCA 21 // Yellow wire
#define ENCB 20 // White wire
#define RPWM 9
#define LPWM 8
#define EN_MOTOR1 49
*/

int pos = 0; 
volatile int posi = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//PID setup

double kp = 1.0, ki = 0.0, kd = 0.02;
double input = 0.0, output = 0.0, setpoint = 0.0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(EN_MOTOR1,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder_right,RISING);
  Startmotor_right(0,0);
  digitalWrite(EN_MOTOR1, LOW);
}

void loop() {
  
 // set target position
  //int target = 1000;
  int target = 1000*sin(prevT/1e6);

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
  else{
    dir = -1;
    }

  // signal the motor
  Startmotor_right(dir,pwr);


  // store previous error
  eprev = e;
  
  //Serial.print(pwr);
  //Serial.print(" ");
  //Serial.print(target);
  //Serial.print(" ");
  Serial.println(pos);
  //Serial.print(" ");
  //Serial.print(dir);
  //Serial.println();




}

void Startmotor_left(int dir, int pwm){
  digitalWrite(EN_MOTOR1, HIGH);
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
  digitalWrite(EN_MOTOR1, LOW);
  }
}

void Startmotor_right(int dir, int pwm){
  digitalWrite(EN_MOTOR1, HIGH);
  if(dir == 1){
    analogWrite(LPWM,pwm);
    analogWrite(RPWM,0);
    }
  else if(dir == -1){
    analogWrite(LPWM,0);
    analogWrite(RPWM,pwm);
    }
  else{
  analogWrite(LPWM,0);
  analogWrite(RPWM,0);
  digitalWrite(EN_MOTOR1, LOW);
  }
}

void readEncoder_left(){
  int b = digitalRead(ENCB);
  if(b>0){
    posi--;
    }
  else{
    posi++;
  }
}  

void readEncoder_right(){
  int b = digitalRead(ENCB);
  if(b>0){
    posi++;
    }
  else{
    posi--;
  }
} 
