#include <iostream>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>

#define _USE_MATH_DEFINES

using namespace std;

const float degree_offset = 320;

bool is_exist = false;

std_msgs::Float32MultiArray degree_msgs;
ros::Publisher ob_degree_pub;

void find_cb(const std_msgs::Bool::ConstPtr &input){
    is_exist = input->data;
}


void camera_cb(const vision_msgs::Detection2DArray::ConstPtr &input){
    float x_center_cord = -1; // -1로 초기화하여 구분
    float x_min_cord = -1; // -1로 초기화하여 구분
    float x_max_cord = -1; // -1로 초기화하여 구분

    float target_x_size = -1;

    float center_degree = 0;
    float min_degree = 0;
    float max_degree = 0;
    float max_x_size = -999;

////////////////////////////////

    const int cord_dif = 50;

    float f_size = -1;
    float s_size = -1;

    bool found_person = false;
    bool no_judge = false;


    for( int i = 0; i < input->detections.size(); i++){
        if(input->detections[i].results[0].id == 1){
            found_person = true;
            
            float x_distance = (input->detections[i].bbox.size_x) / 2;
            x_center_cord = input->detections[i].bbox.center.x;
            x_min_cord =  input->detections[i].bbox.center.x - x_distance;
            x_max_cord =  input->detections[i].bbox.center.x + x_distance;
            target_x_size = input->detections[i].bbox.size_x;
	    }

        if(f_size == -1 && s_size == -1 && f_size < target_x_size){
            f_size = target_x_size;

            center_degree = x_center_cord;
            min_degree = x_min_cord;
            max_degree = x_max_cord;
        }
        else if(f_size != -1 && s_size == -1 && f_size < target_x_size){
            s_size = f_size;
            f_size = target_x_size;

            if(f_size - s_size < cord_dif){
                no_judge = true;
            }            

            else{
                center_degree = x_center_cord;
                min_degree = x_min_cord;
                max_degree = x_max_cord;
            }
        }
        else if(f_size != -1 && s_size != -1 && f_size < target_x_size){
            s_size = f_size;
            f_size = target_x_size;
            
            if(f_size - s_size < cord_dif){
                no_judge = true;
            }
            else
            {
                center_degree = x_center_cord;
                min_degree = x_min_cord;
                max_degree = x_max_cord;
            }

        }
}

    if(found_person && !no_judge){

        float degree = atan2((center_degree-degree_offset)*tan(39*M_PI/180), 320);
        float degree2 = atan2((min_degree-degree_offset)*tan(39*M_PI/180), 320);
        float degree3 = atan2((max_degree-degree_offset)*tan(39*M_PI/180), 320);

        degree_msgs.data.clear();
        degree_msgs.data.push_back(degree * 180/M_PI);
        degree_msgs.data.push_back(degree2 * 180/M_PI);
        degree_msgs.data.push_back(degree3 * 180/M_PI);
    }

    else{

        degree_msgs.data.clear();
        degree_msgs.data.push_back(-999);
        degree_msgs.data.push_back(-999);
        degree_msgs.data.push_back(-999);
    }


}

int main(int argc, char **argv){
    ros::init(argc, argv, "camera_degree");
    ros::NodeHandle nh;
    ros::Subscriber camera_info = nh.subscribe("/detectnet/detections", 10, camera_cb);
    ros::Subscriber object_exist = nh.subscribe("/detectnet/find", 10, find_cb);



    ob_degree_pub = nh.advertise<std_msgs::Float32MultiArray>("ob_degree", 2);

    ros::Rate loop_rate(100);

    while(ros::ok()){
        if(is_exist){
            ob_degree_pub.publish(degree_msgs);
        }
        else{
            degree_msgs.data.clear();
            degree_msgs.data.push_back(-999);
            degree_msgs.data.push_back(-999);
            degree_msgs.data.push_back(-999);
            ob_degree_pub.publish(degree_msgs);
        }
        ros::spinOnce();
		loop_rate.sleep();
    }

}
