#ifndef _RASPBOT_BASE_H_
#define _RASPBOT_BASE_H_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "serial/serial.h"

namespace raspbot
{

class BotBase
{

public:
    BotBase(ros::NodeHandle nh,ros::NodeHandle private_nh);
    ~BotBase();

    void loop();
    int serial_init();

    void raspbotTwistCallBack(const geometry_msgs::Twist::ConstPtr &msg_ptr);

    ros::NodeHandle   nh_;
    ros::NodeHandle   nhPrivate_;
    ros::Publisher    odom_pub;
    ros::Subscriber   twist_sub;
    serial::Serial    sp;

    std::string       base_frame;
    std::string       odom_frame;
    std::string       udev_port;
    int               baud;

};



};



#endif