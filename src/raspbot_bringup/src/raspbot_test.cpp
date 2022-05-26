#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <cstdlib>
#include <ctime>
#include "geometry_msgs/Twist.h"

void randomPubTwist(ros::Publisher &pub);

int main(int argc, char **argv)
{
	ros::init(argc,argv,"raspbot_test_node");
    ros::NodeHandle nh("");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Rate r(20);

    while(ros::ok)
    {
        randomPubTwist(pub);
        r.sleep();
        ros::spinOnce();
    }

}

void randomPubTwist(ros::Publisher &pub)
{
    std::srand(time(nullptr));

    float speed = (std::rand()%(10000+10000)-10000)/1000.0;
    float yaw = (std::rand()%(10000+10000)-10000)/1000.0;

    geometry_msgs::Twist twist;

    twist.linear.x=speed;
    twist.linear.y=0;
    twist.linear.z=0;

    twist.angular.x=0;
    twist.angular.y=0;
    twist.angular.z=yaw;

    pub.publish(twist);

}