#define TRIG1 12
#define ECHO1 11
#define TRIG2 3
#define ECHO2 2

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
std_msgs::Float32 distance1_;  //back
std_msgs::Float32 distance2_;  //left

std_msgs::Float32 duration1_;
std_msgs::Float32 duration2_;

ros::Publisher distancePublisher1("/distance1", &distance1_);
ros::Publisher distancePublisher2("/distance2", &distance2_);


void setup()
{
  nh.initNode();
  nh.advertise(distancePublisher1);
  nh.advertise(distancePublisher2);
  Serial.begin(57600);
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
}

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

void loop()
{
  // Sensor 1: back
  distance1_.data = measureDistance(TRIG1, ECHO1);
  if (distance1_.data >= 500 || distance1_.data <= 0) {
    Serial.println("Sensor1: Out of range"); //duration1
  } else {
    Serial.print("Sensor1: ");
    Serial.print(distance1_.data);
    Serial.println(" cm");
  }
  distancePublisher1.publish(&distance1_);
  delay(100); 

  // Sensor 2: left
  distance2_.data = measureDistance(TRIG2, ECHO2);
  if (distance2_.data >= 500 || distance2_.data <= 0) {
    Serial.println("Sensor2: Out of range"); //duration2
  } else {
    Serial.print("Sensor2: ");
    Serial.print(distance2_.data);
    Serial.println(" cm");
  }
  distancePublisher2.publish(&distance2_);
  delay(100);
  
  nh.spinOnce();
  delay(100);
}
