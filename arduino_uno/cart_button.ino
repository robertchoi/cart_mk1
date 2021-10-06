#include <ros.h>
#include <std_msgs/Int8.h>


ros::NodeHandle nh;
std_msgs::Int8 button;


const int e_stop = -1;

const int joy_control = 0;
const int cart_record = 1;
const int cart_play = 2;
const int stack_init = 3;

#define ESTOP 6
#define DELAY 100

int p[4] = {0};
//int in_pins[4] = {2, 4, 7, 9};
int in_pins[4] = {9, 7, 4, 2};
//int out_pins[4] = {3, 5, 8, 10};
int out_pins[4] = {10, 8, 5, 3};
int state = 0; 
int start = 0;

void sub_ready(const std_msgs::Int8 &msg){
  start = msg.data;
}


ros::Publisher pubButton("button", &button);
ros::Subscriber<std_msgs::Int8> sub("/ready", &sub_ready);

void setup() {
    nh.initNode();
    nh.advertise(pubButton);
    nh.subscribe(sub);
    
    pinMode(out_pins[0] , OUTPUT);
    pinMode(in_pins[0] , INPUT);

    pinMode(out_pins[1] , OUTPUT);
    pinMode(in_pins[1] , INPUT);

    pinMode(out_pins[2] , OUTPUT);
    pinMode(in_pins[2] , INPUT);

    pinMode(out_pins[3] , OUTPUT);
    pinMode(in_pins[3] , INPUT);

    pinMode(ESTOP, INPUT);
    digitalWrite(ESTOP, HIGH);

    Serial.begin(57600);
}

   
void loop(){
  Serial.println(start);
  if(start){
    if (digitalRead(ESTOP) == LOW) { //red button
      Serial.println("ESTOP!");
      button.data = e_stop;
      pubButton.publish(&button);
    } else {
      for(int i=0;i<4;i++) {
        int cur = digitalRead(in_pins[i]);
        if((p[i] == 0) && (cur == 1)) {
          if(state == 0) state = i;
          else if(state == i) state = 0;
          else state = i;       
        }
        p[i] = cur;
      }
      for(int  i=0;i<4;i++) {
        if(state == i) {
          digitalWrite(out_pins[i], HIGH);
          button.data = i;
          pubButton.publish(&button);
        }
        else {
          digitalWrite(out_pins[i], LOW);
          pubButton.publish(&button);
        }
      }
    }
  }
    delay(DELAY);
    nh.spinOnce();
}
