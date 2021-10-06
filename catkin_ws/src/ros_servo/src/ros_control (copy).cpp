#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <ros/time.h>

#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

#include <sensor_msgs/Joy.h>
#include <vision_msgs/Detection2DArray.h>

#include "ros_servo/CartControl.h"

#include <sos_fusion/MinMax.h>
#include <sos_fusion/PointArray.h>

using namespace std;

enum E_STATE
{
	S_STOP = 0,
	S_FORWARD,
	S_BACKWARD,
	S_RIGHT,
	S_LEFT,
	S_FORWARDRIGHT,
	S_FORWARDLEFT,
	S_BACKWARDRIGHT,
	S_BACKWARDLEFT,
	S_NONE,
	S_BOOZER,
	S_TEMP_NONE
};

enum E_MODE
{
	M_ESTOP = -1,
	M_JOYSTICK = 0,
	M_SELF_DRIVING,
	M_SELF_DRIVING_BACK,
	M_STACK_RESET
};

ros::Publisher cart_state_pub;
ros::Publisher moving_pub;

ros_servo::CartControl cart_state_msg;
std_msgs::Bool moving_msg;


stack<ros_servo::CartControl> state_stk; // store msg
stack<ros_servo::CartControl> out_state_stk; // play back msg

E_STATE state = S_NONE;	  // not find person - by lidar
E_MODE button = M_JOYSTICK;


bool person_find = false; // not find person - by camera

bool moving = true;
bool isFirst = true;

bool stop = false;

float cart_length = 1.0f;

sos_fusion::MinMax person;
sos_fusion::MinMax person_back;

sos_fusion::PointArray Obstacle;
sos_fusion::PointArray Obstacle_back;

const int SPEED_MIN = 95;
const int TURN_SPEED_MIN = 20;
const int X_MULTIPLIER = 20;
const float ANGLE_MULTIPLIER = 0.3f;


const float X_FAR = 0.6f;
const float X_CLOSE = 0.30f;
const float ANGLE_TO_TURN = 13.0f;

const int X_BACK_MULTIPLIER = 8;
const float X_BACK_FAR = -0.6f - cart_length;
const float X_BACK_CLOSE = -0.30f - cart_length;
const float ANGLE_TO_TURN_BACK = 6.0f;


uint16_t left_speed = 0;
uint16_t right_speed = 0;

const float JOY_X_MAX = 1.0;
const float JOY_X_HIGH = 0.5;
const float JOY_X_DEFAULT = 0.0;
const float JOY_X_LOW = -0.5;
const float JOY_X_MIN = -1.0;
const float JOY_Y_MAX = 1.0;
const float JOY_Y_HIGH = 0.5;
const float JOY_Y_DEFAULT = 0.0;
const float JOY_Y_LOW = -0.5;
const float JOY_Y_MIN = -1.0;


float joy_x = 0;
float joy_y = 0;

//////////////////////////////////////////

const int JOY_X_OFF = 522;
const int JOY_Y_OFF = 492;

const int JOY_X_RATIO = 501;
const int JOY_Y_RATIO = 531;

const float JOY_MULTIPLIER = 210.0f; // 225
const float JOY_DECREASE = 0.6f;

//////////////////////////////////////////

int d_insert = 0;

int front_change_cnt = 0;
int back_change_cnt = 0;
int person_cnt = 500;

int upper_limit = 500;

void self_driving(float x, float angle, E_STATE state);

#pragma region Callbacks

void find_cb(const std_msgs::Float32MultiArray::ConstPtr& input)
{
	if (input->data[0] != -999){
		person_find = true;
		person_cnt = 0;
	}
	else{
		person_find = false;
		person_cnt++;
	}		
}

void joy_cb(const sensor_msgs::Joy::ConstPtr& input)
{
	joy_x = input->axes[0];
	joy_y = input->axes[1];
}

void button_cb(const std_msgs::Int8::ConstPtr& input)
{
	button = (E_MODE)input->data;
}

void obstacle_cb(const sos_fusion::PointArray::ConstPtr& input)
{
	Obstacle = *input;
}

void obstacle_back_cb(const sos_fusion::PointArray::ConstPtr& input)
{
	Obstacle_back = *input;
}

void person_track_cb(const sos_fusion::MinMax::ConstPtr& input) {
	person = *input;
}

void person_track_back_cb(const sos_fusion::MinMax::ConstPtr& input) {
	person_back = *input;
}

bool can_move(const geometry_msgs::Point &cen, const geometry_msgs::Point &min, const geometry_msgs::Point &max, int mode) {
	float side = 0.30f;
	float front = 0.15f;
	
	float behind = -(front + cart_length);

    if(mode == 1){
    	if (min.x <= front) {
    		if (cen.y > 0 && min.y <= side)
    			return false;
    		else if (cen.y <= 0 && max.y >= -side)
    			return false;
    	}
    	return true;
    }
    else if(mode == 2){
    	if (behind <= min.x) {

    		if (cen.y > 0 && min.y <= side)
    			return false;
    		else if (cen.y <= 0 && max.y >= -side)
    			return false;
    	}
    	return true;
    }
}

void judge_move(const sos_fusion::PointArray &front_input, const sos_fusion::PointArray &back_input){
	moving = true;
	for (int i = 0; i < front_input.cluster_center.size(); i++) {
		moving *= can_move(front_input.cluster_center[i], front_input.cluster_min[i], front_input.cluster_max[i], 1);
	}

	for (int i = 0; i < back_input.cluster_center.size(); i++) {
		moving *= can_move(back_input.cluster_center[i], back_input.cluster_min[i], back_input.cluster_max[i], 2);
	}	

	moving_msg.data = moving;
}


#pragma endregion

void cart_judge(float x, float y)
{
	float angle = atan(y / x) * (180.0f / M_PI);

	if(x >= 0){
		state = (X_CLOSE < x && x < X_FAR && -ANGLE_TO_TURN < angle && angle < ANGLE_TO_TURN) ? S_STOP : 
		(x < X_CLOSE && -ANGLE_TO_TURN < angle && angle < ANGLE_TO_TURN) ? S_BACKWARD : 
		(X_CLOSE < x && x < X_FAR && angle > ANGLE_TO_TURN) ? S_LEFT : 
		(X_CLOSE < x && x < X_FAR && angle < -ANGLE_TO_TURN) ? S_RIGHT : 
		(x > X_FAR && angle > ANGLE_TO_TURN) ? S_FORWARDLEFT : 
		(x > X_FAR && angle < -ANGLE_TO_TURN) ? S_FORWARDRIGHT : 
		(x < X_CLOSE && angle < -ANGLE_TO_TURN) ? S_BACKWARDLEFT : 
		(x < X_CLOSE && angle > ANGLE_TO_TURN) ? S_BACKWARDRIGHT : 
		(x > X_FAR  && -ANGLE_TO_TURN < angle && angle < ANGLE_TO_TURN) ? S_FORWARD : S_NONE;
	
		self_driving(x, angle, state);
	}

	else{
		state = (X_BACK_FAR < x && x < X_BACK_CLOSE && -ANGLE_TO_TURN_BACK < angle && angle < ANGLE_TO_TURN_BACK) ? S_STOP : 
		(X_BACK_CLOSE < x && -ANGLE_TO_TURN_BACK < angle && angle < ANGLE_TO_TURN_BACK) ? S_FORWARD : 
		(x < X_BACK_CLOSE && X_BACK_FAR < x && angle > ANGLE_TO_TURN_BACK) ? S_LEFT : 
		(x < X_BACK_CLOSE && X_BACK_FAR < x && -ANGLE_TO_TURN_BACK < angle) ? S_RIGHT : 
		(x < X_BACK_FAR && angle > ANGLE_TO_TURN_BACK) ? S_FORWARDLEFT : 
		(x < X_BACK_FAR && angle < -ANGLE_TO_TURN_BACK) ? S_FORWARDRIGHT : 
		(x > X_BACK_CLOSE && angle < -ANGLE_TO_TURN) ? S_BACKWARDLEFT : 
		(x > X_BACK_CLOSE && angle > ANGLE_TO_TURN) ? S_BACKWARDRIGHT : 
		(x < X_BACK_FAR  && -ANGLE_TO_TURN < angle && angle < ANGLE_TO_TURN) ? S_BACKWARD : S_NONE;

		self_driving(-(x + cart_length), angle, state);
	}
}

void self_driving(float x, float angle, E_STATE state)
{
	switch (state)
	{
	case S_STOP:
		left_speed = 0;
		right_speed = 0;
		break;
	case S_FORWARD:
		left_speed = SPEED_MIN + (int)(x * X_MULTIPLIER);
		right_speed = SPEED_MIN + (int)(x * X_MULTIPLIER);
		break;
	case S_BACKWARD:
		left_speed = SPEED_MIN + (int)(x * X_BACK_MULTIPLIER);
		right_speed = SPEED_MIN + (int)(x * X_BACK_MULTIPLIER);
		break;
	case S_RIGHT:
		left_speed = (x * 10) + SPEED_MIN + (int)(x * X_MULTIPLIER) + (int)(abs(angle) * ANGLE_MULTIPLIER);
		right_speed = (x * 10) + SPEED_MIN + (int)(x * X_MULTIPLIER) + (int)(abs(angle) * ANGLE_MULTIPLIER);
		break;
	case S_LEFT:
		left_speed = (x * 10) + SPEED_MIN + (int)(x * X_MULTIPLIER) + (int)(abs(angle) * ANGLE_MULTIPLIER);
		right_speed = (x * 10) + SPEED_MIN + (int)(x * X_MULTIPLIER) + (int)(abs(angle) * ANGLE_MULTIPLIER);
		break;
	case S_FORWARDRIGHT:
		left_speed = TURN_SPEED_MIN + SPEED_MIN + (int)(x * X_MULTIPLIER) + (int)(abs(angle) * ANGLE_MULTIPLIER);
		right_speed = TURN_SPEED_MIN + SPEED_MIN + (int)(x * X_MULTIPLIER);
		break;
	case S_FORWARDLEFT:
		left_speed = TURN_SPEED_MIN + SPEED_MIN + (int)(x * X_MULTIPLIER);
		right_speed = TURN_SPEED_MIN + SPEED_MIN + (int)(x * X_MULTIPLIER) + (int)(abs(angle) * ANGLE_MULTIPLIER);
		break;
	case S_BACKWARDRIGHT:
		left_speed = TURN_SPEED_MIN + SPEED_MIN + (int)(x * X_MULTIPLIER) + (int)(abs(angle) * ANGLE_MULTIPLIER);
		right_speed = TURN_SPEED_MIN + SPEED_MIN + (int)(x * X_MULTIPLIER);
		break;
	case S_BACKWARDLEFT:
		left_speed = TURN_SPEED_MIN + SPEED_MIN + (int)(x * X_MULTIPLIER);
		right_speed = TURN_SPEED_MIN + SPEED_MIN + (int)(x * X_MULTIPLIER) + (int)(abs(angle) * ANGLE_MULTIPLIER);
		break;
	case S_NONE:
		left_speed = 0;
		right_speed = 0;
		break;
	default:
		cout <<"[Warning] wrong state input\n";
		break;
	}
}

void joy_driving(float x, float y)
{
	state = (JOY_X_LOW <= x && x <= JOY_X_HIGH && JOY_Y_LOW <= y && y <= JOY_Y_HIGH) ? S_STOP : 
	(JOY_X_LOW <= x && x <= JOY_X_HIGH && JOY_Y_HIGH <= y && y <= JOY_Y_MAX) ? S_FORWARD : 
	(JOY_X_LOW <= x && x <= JOY_X_HIGH && JOY_Y_MIN <= y && y <= JOY_Y_LOW) ? S_BACKWARD : 
	(JOY_X_MIN <= x && x <= JOY_X_LOW && JOY_Y_LOW <= y && y <= JOY_Y_HIGH) ? S_RIGHT : 
	(JOY_X_HIGH <= x && x <= JOY_X_MAX && JOY_Y_LOW <= y && y <= JOY_Y_HIGH) ? S_LEFT : 
	(JOY_X_MIN <= x && x <= JOY_X_LOW && JOY_Y_HIGH <= y && y <= JOY_Y_MAX) ? S_FORWARDRIGHT : 
	(JOY_X_HIGH <= x && x <= JOY_X_MAX && JOY_Y_HIGH <= y && y <= JOY_Y_MAX) ? S_FORWARDLEFT : 
	(JOY_X_MIN <= x && x <= JOY_X_LOW && JOY_Y_MIN <= y && y <= JOY_Y_LOW) ? S_BACKWARDRIGHT : 
	(JOY_X_HIGH <= x && x <= JOY_X_MAX && JOY_Y_MIN <= y && y <= JOY_Y_LOW) ? S_BACKWARDLEFT : S_NONE;


	switch(state){
		case S_FORWARD:
		case S_BACKWARD:
			left_speed = right_speed = (int)(abs(y) * JOY_MULTIPLIER);
			break;	
		case S_LEFT:
		case S_RIGHT:
			left_speed = right_speed = (int)(abs(x) * JOY_MULTIPLIER);
			break;
		case S_FORWARDRIGHT:
		case S_FORWARDLEFT:
		case S_BACKWARDRIGHT:	
		case S_BACKWARDLEFT:
			left_speed = right_speed = ((int)(abs(x) * JOY_MULTIPLIER) + (int)(abs(y) * JOY_MULTIPLIER)) * JOY_DECREASE;
			break;
		default:
			left_speed = right_speed = 0;
			break;
	}

}


void cart_moving()
{
	if (button == M_ESTOP){
		cart_state_msg.state = S_STOP;
		cart_state_msg.left_speed = 0;
		cart_state_msg.right_speed = 0;

	}
	
	else if (button == M_JOYSTICK)
	{
		joy_driving(joy_x, joy_y);

		cart_state_msg.state = state;
		cart_state_msg.left_speed = left_speed;
		cart_state_msg.right_speed = right_speed;
		
		front_change_cnt = 0;
		back_change_cnt = 0;
	}

	else if (button == M_SELF_DRIVING)
	{
		if(front_change_cnt <= 1)
			front_change_cnt++;

		if(front_change_cnt == 1){
			ros::Duration(5,0).sleep();
		}

		judge_move(Obstacle, Obstacle_back);

		// If camera found a person
		if (person_find)
		{			
			if (person.cen.x != person.min.x && person.cen.x != person.max.x)
			{
				if(moving)
					cart_judge(person.cen.x, person.cen.y);
				else{
					state = S_BOOZER;
					left_speed = 0;
					right_speed = 0;
				}
					
			}
			else
			{
				state = S_TEMP_NONE;
				left_speed = 0;
				right_speed = 0;
			}
		}
		else
		{
			if(person_cnt >= upper_limit){
				state = S_NONE;
				left_speed = 0;
				right_speed = 0;
			}
			else{
				state = S_TEMP_NONE;
				left_speed = 0;
				right_speed = 0;
			}
		}

		cart_state_msg.state = state;
		cart_state_msg.left_speed = left_speed;
		cart_state_msg.right_speed = right_speed;
		back_change_cnt = 0;
	}
	else if (button == M_SELF_DRIVING_BACK)
	{	
		if(back_change_cnt <= 1)
			back_change_cnt++;

		if(back_change_cnt == 1){
			ros::Duration(5,0).sleep();
		}
	

		judge_move(Obstacle, Obstacle_back);
		// If camera found a person
		if (person_find)
		{			
			if (person_back.cen.x != person_back.min.x && person_back.cen.x != person_back.max.x)
			{
				if(moving)
					cart_judge(person_back.cen.x, person_back.cen.y);
				else{
					state = S_BOOZER;
					left_speed = 0;
					right_speed = 0;
				}
					
			}
			else
			{
				state = S_TEMP_NONE;
				left_speed = 0;
				right_speed = 0;
			}
		}
		else
		{
			if(person_cnt >= upper_limit){
				state = S_NONE;
				left_speed = 0;
				right_speed = 0;
			}
			else{
				state = S_TEMP_NONE;
				left_speed = 0;
				right_speed = 0;
			}
		}
		cart_state_msg.state = state;
		cart_state_msg.left_speed = left_speed;
		cart_state_msg.right_speed = right_speed;
		front_change_cnt = 0;
	}
	else 
	{
		front_change_cnt = 0;
		back_change_cnt = 0;
		return;
	}
}


int main(int argc, char **argv)
{
	//ros::Time::init();
	ros::init(argc, argv, "ros_control");
	ros::NodeHandle nh;
	ros::Subscriber person_sub = nh.subscribe("/target_object_position", 5, person_track_cb);
	ros::Subscriber obstacle_sub = nh.subscribe("/objects_positions", 5, obstacle_cb);


	ros::Subscriber person_back_sub = nh.subscribe("/target_object_back_position", 5, person_track_back_cb);
	ros::Subscriber obstacle_back_sub = nh.subscribe("/objects_back_positions", 5, obstacle_back_cb);



	ros::Subscriber camera_sub = nh.subscribe("/ob_degree", 5, find_cb);
	
    	ros::Subscriber joy_stick_sub = nh.subscribe("/joy", 5, joy_cb);

	ros::Subscriber button_sub = nh.subscribe("/button", 10, button_cb);

	cart_state_pub = nh.advertise<ros_servo::CartControl>("cart_state", 100);

	moving_pub = nh.advertise<std_msgs::Bool>("moving", 1);

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		cart_moving();
		cart_state_pub.publish(cart_state_msg);
		moving_pub.publish(moving_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

