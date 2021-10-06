#include<cmath>

#include<ros/ros.h>
#include<std_msgs/UInt16.h>
#include <std_msgs/Int8.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

#include<sos_fusion/MinMax.h>

using namespace std;

#define _USE_MATH_DEFINES

enum E_MODE
{
	M_ESTOP = -1,
	M_JOYSTICK = 0,
	M_SELF_DRIVING,
	M_SELF_DRIVING_BACK,
	M_STACK_RESET
};

E_MODE button = M_JOYSTICK;

float length = 6;

float degree;
int degree_cnt = -1;

ros::Publisher line_pub;
ros::Publisher text_pub;
ros::Publisher bbox_pub;

geometry_msgs::Point min_point;
geometry_msgs::Point max_point;

#pragma region callback

void degree_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
    degree_cnt = (degree_cnt + 1) % 10; 

    degree = (msg->data[0]) + 90;
    cout<<"degree = "<< degree <<endl;

}


void person_cb(const sos_fusion::MinMax::ConstPtr& input){
    min_point = input->min;
    max_point = input->max;
}


void button_cb(const std_msgs::Int8::ConstPtr& input)
{
	button = (E_MODE)input->data;
}

#pragma endregion callback


void printLine(){
    visualization_msgs::Marker line_strip;

    line_strip.header.frame_id = "/laser";
    line_strip.header.stamp  = ros::Time::now();
    line_strip.ns = "line";
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.lifetime = ros::Duration();
    
    line_strip.scale.x = 0.1;


    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    geometry_msgs::Point sp;
    sp.x = 0;
    sp.y = 0;
    sp.z = 0;

    geometry_msgs::Point ep;
    ep.x = -length*sin(degree*M_PI/180);
    ep.y = -length*cos(degree*M_PI/180);
    ep.z = 0;

    line_strip.points.push_back(sp);
    line_strip.points.push_back(ep);
    line_pub.publish(line_strip);
}


void printText(){
    visualization_msgs::Marker line_label;

    line_label.header.frame_id = "/laser";
    line_label.header.stamp  = ros::Time::now();
    line_label.ns = "text";



    line_label.pose.position.x = -length*sin(degree*M_PI/180);
    line_label.pose.position.y = -length*cos(degree*M_PI/180);
    line_label.pose.position.z = 1.0;

    
    line_label.pose.orientation.x = 0.0;
    line_label.pose.orientation.y = 0.0;
    line_label.pose.orientation.z = 0.0;
    line_label.pose.orientation.w = 1.0;

    line_label.id = 1;

    line_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    line_label.action = visualization_msgs::Marker::ADD;
    line_label.lifetime = ros::Duration();

    line_label.scale.z = 1.0;

    line_label.text = "person";

    line_label.color.r = 0.0f;
    line_label.color.g = 1.0f;
    line_label.color.b = 0.0f;
    line_label.color.a = 1.0;


    text_pub.publish(line_label);
}


void printBox(){

    visualization_msgs::Marker bbox;
    bbox.header.frame_id = "/laser";
    bbox.header.stamp  = ros::Time::now();
    bbox.ns = "bbox";
    bbox.pose.orientation.w = 1.0;
    bbox.id = 3;

    bbox.type = visualization_msgs::Marker::LINE_STRIP;
    bbox.action = visualization_msgs::Marker::ADD;
    bbox.lifetime = ros::Duration();

    bbox.scale.x = 0.06;

    bbox.color.r = 1.0;
    bbox.color.g = 0.0;
    bbox.color.b = 0.0;
    bbox.color.a = 0.7f;


    for(int i = 0; i < 5; i++){
        geometry_msgs::Point p;

        switch(i){
            case 0:
                p.x = min_point.x;
                p.y = min_point.y;
                break;
            case 1:
                p.x = max_point.x;
                p.y = min_point.y;
                break;
            case 2:
                p.x = max_point.x;
                p.y = max_point.y;
                break;
            case 3:
                p.x = min_point.x;
                p.y = max_point.y;
                break;
            case 4:
                p.x = min_point.x;
                p.y = min_point.y;
                break;
        }

        bbox.points.push_back(p);
    }
    bbox_pub.publish(bbox);
}


int main(int argc, char** argv){

    ros::init(argc, argv, "fusion_back");
    ros::NodeHandle nh;



    ros::Subscriber deg_sub = nh.subscribe("/ob_degree", 10, degree_cb);
    ros::Subscriber person_sub = nh.subscribe("/target_object_back_position", 10, person_cb);
    ros::Subscriber button_sub = nh.subscribe("/button", 10, button_cb);

    line_pub = nh.advertise<visualization_msgs::Marker>("/line_back_pub",10);
    text_pub = nh.advertise<visualization_msgs::Marker>("/text_back_pub",10);
    bbox_pub = nh.advertise<visualization_msgs::Marker>("/bbox_back_pub",10);


    ros::Rate loop_rate(100);

    while(ros::ok()){
        if(degree_cnt != -1 && degree != -909 && button == M_SELF_DRIVING_BACK){
            printLine();
            printText();
            printBox();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
