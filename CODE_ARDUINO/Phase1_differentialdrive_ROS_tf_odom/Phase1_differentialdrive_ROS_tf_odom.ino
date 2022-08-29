//1m = 5850 encoder counts (still to reconfirm)

#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <PID_v1.h>

///cmd_vel variables from ROS
float demandx;
float demandz;

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

//** ROS Callback & Subscriber **
void velCallback( const geometry_msgs::Twist& vel)
  {
    demandx = vel.linear.x;
    demandz = vel.angular.z;
    //demandx = constrain(demandx, -0.25,0.25); //constrain the max speed
    //demandx = constrain(demandx, -1,1);
  }


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback); //Create a subscriber for ROS cnd_vel Topic
/*
geometry_msgs::Vector3Stamped speed_msg; //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg); //create a publisher to ROS topic "speed" using the "speed_msg" type
*/

//LIPO protection
const int BatteryPin = A0;
const int Buzzer = 10;
float LIPO_volt;

//PID Values for Right Wheel
double Pk0 = 1;
double Ik0 = 0;
double Dk0 = 0;

double Setpoint0, Input0, Output0, Output0a;
PID PID0(&Input0, &Output0, &Setpoint0, Pk0, Ik0, Dk0, DIRECT);

//PID Values for Left Wheel
double Pk1 = 1;
double Ik1 = 0;
double Dk1 = 0;

double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);


float demand0;
float demand1;

unsigned long currentMillis;
unsigned long previousMillis;

///////ROS
//tf
double x = 0;
double y = 0;
double theta = 0;

char base_link[] = "base_link";
char odom[] = "odom";

float pos0_mm_diff;
float pos1_mm_diff;
float pos_avg_mm_diff;
float pos_total_mm;


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

  pinMode(Buzzer, OUTPUT); //Burzzer for LIPO protection
  
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
  //Serial1.begin(115200);
  
  //ROS
  nh.getHardware() -> setBaud(115200);
  nh.initNode(); //initialise ROS
  nh.subscribe(sub); //subscribe to cmd_vel topic
  nh.advertise(odom_pub); //publish /odom
  broadcaster.init(nh);
}

void loop() {
  
  nh.spinOnce(); //listen to ROS msgs + activate the callback if there is one

  currentMillis= millis();
  if(currentMillis - previousMillis >= 10){ //start of timed loop = 10ms cycle
    previousMillis = currentMillis;


    /*  5650 encodercount for 1m. 56.5 encodercounts met 10 millisecond loop at 1 m/s velocity
     *  56.5 encodercunt per mm.
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

    pos0_mm_diff = encoder0Diff / 56.5;
    pos1_mm_diff = encoder1Diff / 56.5;

    pos_avg_mm_diff = (pos0_mm_diff + pos1_mm_diff)/2;
    pos_total_mm += pos_avg_mm_diff;
    
    //Serial1.print("Right Position: ");
    //Serial1.print(encoder0Pos);
    //Serial1.print(" , ");
    //Serial1.print("Left Position: ");
    //Serial1.print(encoder1Pos);
    //Serial1.println("");

    //PID for Both Wheels
    Setpoint0 = demand0*56.5*5; //added factor 5 to match speed 1 m/s IRL
    Setpoint1 = demand1*56.5*5;
    Input0 = encoder0Diff;
    Input1 = encoder1Diff;
    PID0.Compute();
    PID1.Compute();

    //digitalWrite(EN_MOTOR_LEFT, HIGH);
    //digitalWrite(EN_MOTOR_RIGHT, HIGH);



    if(Output0 > 0 ){
      digitalWrite(EN_MOTOR_RIGHT, HIGH);
      Output0a = abs(Output0);
      analogWrite(Right_LPWM,Output0a);
      analogWrite(Right_RPWM,0);
     }

     else if(Output0 < 0){
      digitalWrite(EN_MOTOR_RIGHT, HIGH);
      Output0a = abs(Output0);
      analogWrite(Right_LPWM,0);
      analogWrite(Right_RPWM,Output0a);

     }
     else {
      digitalWrite(EN_MOTOR_RIGHT, LOW);
      analogWrite(Left_LPWM,0);
      analogWrite(Left_RPWM,0);
      }

     if(Output1 > 0){
      digitalWrite(EN_MOTOR_LEFT, HIGH);
      Output1a = abs(Output1);
      analogWrite(Left_LPWM,0);
      analogWrite(Left_RPWM,Output1a);
     }

     else if(Output1 < 0){
      digitalWrite(EN_MOTOR_LEFT, HIGH);
      Output1a = abs(Output1);
      analogWrite(Left_LPWM,Output1a);
      analogWrite(Left_RPWM,0);
     }
     else {
      digitalWrite(EN_MOTOR_LEFT, LOW);
      analogWrite(Left_LPWM,0);
      analogWrite(Left_RPWM,0);
     }

     //calc angle or rotation for broadcast to tf
     theta  += (pos1_mm_diff - pos0_mm_diff) / 360;

     if(theta > PI){theta -=TWO_PI;}
     if(theta < (-PI)){theta +=TWO_PI;}

     //calc x and y to broadcast with tf
     x += pos_avg_mm_diff * cos(theta*10);
     y += pos_avg_mm_diff * sin(theta*10);

     // *** broadcast odom -> base_link transfrom with tf ***
     geometry_msgs::TransformStamped t;

     t.header.frame_id = odom;
     t.child_frame_id = base_link;

     t.transform.translation.x = x/100;
     t.transform.translation.y = y/100;
     t.transform.translation.z = 0;

     t.transform.rotation = tf::createQuaternionFromYaw(theta*10);
     t.header.stamp = nh.now();

     broadcaster.sendTransform(t);

     // *** broadcast odom mesaage ***
     nav_msgs::Odometry odom_msg;
     odom_msg.header.stamp = nh.now();
     odom_msg.header.frame_id = odom;
     odom_msg.pose.pose.position.x = x/100;
     odom_msg.pose.pose.position.y = y/100;
     odom_msg.pose.pose.position.z = 0.0;
     odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta*10);

     odom_msg.child_frame_id = base_link;
     odom_msg.twist.twist.linear.x = ((pos0_mm_diff + pos1_mm_diff) /2)/2; //forward velocity
     odom_msg.twist.twist.linear.y = 0.0; //robot does not move sideways
     odom_msg.twist.twist.angular.z = ((pos1_mm_diff - pos0_mm_diff)/360)*500; //angular velocity

     odom_pub.publish(&odom_msg);

      
      //Serial1.println("Output Right ");
      //Serial1.println(Output0);
      //Serial1.println("Output Left ");
      //Serial1.println(Output1);
      

      LIPO_volt = (((analogRead(BatteryPin) * 5.0 / 1023.0)/1.2) * (1.2+6.2)); //voltagedivider 1.2k+6.2k resistors
      //Serial.print("LIPO_Volt:");
      //Serial.println(LIPO_volt);
      if(LIPO_volt < 9.6){
        tone(Buzzer, 1000);
        //Serial.println("LOW BATTERY");
      }
      else{noTone(Buzzer);}
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
