#ifndef _RASPBOT_BASE_H_
#define _RASPBOT_BASE_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


namespace raspbot
{

class BotBase
{

public:
    BotBase();
    ~BotBase();


    ros::Publisher    odom_pub;
    ros::Subscriber   twist_sub;



};



};



#endif