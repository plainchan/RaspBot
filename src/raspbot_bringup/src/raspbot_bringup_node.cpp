#include "ros/ros.h"
#include "raspbot_bringup/raspbot_base.h"




int main(int argc,char **argv)
{

    ros::init(argc,argv,"raspbot_bringup_node");
    ros::NodeHandle nh,private_nh("~");
    raspbot::BotBase raspberryBot(nh,private_nh);
    raspberryBot.initialize();
    ros::spin();
    return 0;
}