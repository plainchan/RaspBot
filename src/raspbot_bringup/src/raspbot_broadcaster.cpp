#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <cstdlib>
#include <ctime>
#include "geometry_msgs/Twist.h"


int main(int argc, char **argv)
{
	ros::init(argc,argv,"raspbot_broadcaster_node");
    ros::NodeHandle nh("");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Rate r(20);

    while(ros::ok)
    {

        r.sleep();
        ros::spinOnce();
    }

}

