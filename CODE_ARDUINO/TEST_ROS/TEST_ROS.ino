#include <ros.h>
#include <geometry_msgs/Twist.h>

float x;
float z;

ros::NodeHandle   nh;

void velCallback( const geometry_msgs::Twist& vel){

  x = vel.linear.x;
  z = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);

void setup() {
  //Serial1 for Arduino mega 
  Serial1.begin(115200);
  Serial1.print("Started");
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  Serial1.print(x);
  Serial1.print(" , ");
  Serial1.println(z);
  nh.spinOnce();
  delay(10);

}
