/********************************************
   Name:_2_channel_high_relay
   Description: Control the 2 channel relay module ON or OFF
   Website:www.sunfounder.com
   Email: service@sunfounder.com
*******************************************/

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <ros_servo/CartControl.h>

#define LEFT 9
#define RIGHT 10
#define SPEAKER 12

ros::NodeHandle nh;
std_msgs::String str_msg;

unsigned long cur_t = 0;
unsigned long pre_t = 0;

//the relays connect to pin 3 and pin 4
const int IN1 = 4;
const int IN2 = 3;

int left_speed = 80;
int right_speed = 80;
int state = -1;

const int max_speed = 150;

const float turn_offset_one = 1.0f;
const float turn_offset_two = 0.7f;
const int MAX_SPEED_DELTA = 3;

int ON =  1;
int OFF =  0;

void subtemp(const ros_servo::CartControl &msg) {
  int action;
  action = msg.state;

  left_speed = min(msg.left_speed, max_speed);
  right_speed = min(msg.right_speed, max_speed);
  
  if (action == 0) {
    relay_stop();
  }
  else if (action == 2) {
    relay_back();
  }
  else if (action == 4) {
    relay_left();
  }
  else if (action == 3) {
    relay_right();
  }
  else if (action == 1) {
    relay_str();
  }
  else if (action == 5) {
    //relay_right();
    relay_sright();
  }
  else if (action == 6) {
    relay_sleft();
    //relay_left();
  }
  else if (action == 7) {
    relay_bright();
  }
  else if (action == 8) {
    relay_bleft();
  }
  else if (action == 9) {
    relay_nperson();
  }
  else if (action == 10) {
    relay_obstacle();
  }
  else if (action == 11) {
    relay_temp_nperson();
  }
  else {
    relay_stop();
  }
}


ros::Subscriber<ros_servo::CartControl> sub("cart_state", &subtemp);

ros::Publisher pub("temp", &str_msg);

void setup()
{
  nh.initNode();
  relay_init();//initialize the relay
  Serial.begin(57600); // 57600
  nh.advertise(pub);
  nh.subscribe(sub);
  pre_t = millis();
}

void loop() {
  cur_t = millis();
  
  if(cur_t - pre_t >= 30) {
    
  }
  nh.spinOnce();
  delay(100);
}
void relay_init(void)//initialize the relay
{
  //set all the relays OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, OFF);
  digitalWrite(IN2, OFF);
}
//set the status of relays
void relay_str()
{
  str_msg.data = "straight!";
  pub.publish(&str_msg);
  digitalWrite(IN1, ON);
  digitalWrite(IN2, ON);
  analogWrite(LEFT, left_speed);
  analogWrite(RIGHT, right_speed);
}

void relay_stop()
{
  str_msg.data = "stop!";
  pub.publish(&str_msg);
  digitalWrite(IN1, OFF);
  digitalWrite(IN2, OFF);
  analogWrite(LEFT, 0);
  analogWrite(RIGHT, 0);
}


void relay_back()
{
  str_msg.data = "back!";
  pub.publish(&str_msg);
  digitalWrite(IN1, OFF);
  digitalWrite(IN2, OFF);
  analogWrite(LEFT, left_speed);
  analogWrite(RIGHT, right_speed);
}

void relay_left()
{
  str_msg.data = "left!";
  pub.publish(&str_msg);

  digitalWrite(IN1, OFF);
  digitalWrite(IN2, ON);
  analogWrite(LEFT, left_speed);
  analogWrite(RIGHT, right_speed);
}

void relay_right()
{
  str_msg.data = "right!";
  pub.publish(&str_msg);

  digitalWrite(IN1, ON);
  digitalWrite(IN2, OFF);
  analogWrite(LEFT, left_speed);
  analogWrite(RIGHT, right_speed);
}

void relay_sleft()
{
  str_msg.data = "straingt_left!";
  pub.publish(&str_msg);

  digitalWrite(IN1, OFF);
  digitalWrite(IN2, ON);
  analogWrite(LEFT, left_speed * turn_offset_two);
  analogWrite(RIGHT, right_speed * turn_offset_one);
}

void relay_sright()
{
  str_msg.data = "straight_right!";
  pub.publish(&str_msg);

  digitalWrite(IN1, ON);
  digitalWrite(IN2, OFF);
  analogWrite(LEFT, left_speed * turn_offset_one );
  analogWrite(RIGHT, right_speed * turn_offset_two );
}

void relay_bleft()
{
  str_msg.data = "back_left!";
  pub.publish(&str_msg);

  digitalWrite(IN1, ON);
  digitalWrite(IN2, OFF);
  analogWrite(LEFT, left_speed * turn_offset_two );
  analogWrite(RIGHT, right_speed * turn_offset_one );
}

void relay_bright()
{
  str_msg.data = "back_right!";
  pub.publish(&str_msg);

  digitalWrite(IN1, OFF);
  digitalWrite(IN2, ON);
  analogWrite(LEFT, left_speed * turn_offset_one);
  analogWrite(RIGHT, right_speed * turn_offset_two );
}

void relay_nperson()
{
  str_msg.data = "non_person!";
  pub.publish(&str_msg);
  digitalWrite(IN1, OFF);
  digitalWrite(IN2, OFF);
  analogWrite(LEFT, 0);
  analogWrite(RIGHT, 0);
  
  tone(SPEAKER, 450, 50);
  delay(50);
}

void relay_temp_nperson()
{
  str_msg.data = "temp_non_person!";
  pub.publish(&str_msg);
  digitalWrite(IN1, OFF);
  digitalWrite(IN2, OFF);
  analogWrite(LEFT, 0);
  analogWrite(RIGHT, 0);
  
  tone(SPEAKER, 300, 50);
  delay(50);
  tone(SPEAKER, 300, 50);
  delay(50);
  tone(SPEAKER, 300, 50);
  delay(100);

}

void relay_obstacle()
{
  str_msg.data = "ob_detected!";
  pub.publish(&str_msg);
  digitalWrite(IN1, OFF);
  digitalWrite(IN2, OFF);
  analogWrite(LEFT, 0);
  analogWrite(RIGHT, 0);

  tone(SPEAKER, 300, 50);
  delay(100);
}
