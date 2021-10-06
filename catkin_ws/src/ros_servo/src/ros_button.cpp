#include <ros/ros.h>
#include <std_msgs/Int8.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "ready");

    ros::NodeHandle n;
    ros::Publisher ready_pub = n.advertise<std_msgs::Int8>("ready", 1);

    ros::Rate loop_rate(100);

    while(ros::ok()){
        std_msgs::Int8 msg;

        msg.data = 1;

        ready_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}