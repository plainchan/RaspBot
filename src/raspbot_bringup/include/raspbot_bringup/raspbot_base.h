#ifndef _RASPBOT_BASE_H_
#define _RASPBOT_BASE_H_

#include "ros/ros.h"
#include "serial/serial.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

namespace raspbot
{

class BotBase
{

public:
    BotBase(ros::NodeHandle nh,ros::NodeHandle private_nh);
    ~BotBase();

    void initialize();
    bool serial_init();

    void speedTwistCallBack(const geometry_msgs::Twist::ConstPtr &msg_ptr);
    
    void setting();
    void periodicUpdate(const ros::TimerEvent &event);
    
protected:
    ros::NodeHandle   nh_;
    ros::NodeHandle   nhPrivate_;

    ros::Publisher    odom_pub_;
    ros::Publisher    imu_pub_;

    ros::Subscriber   twist_sub_;

    serial::Serial    sp_;


    std::string       base_frame_;
    std::string       odom_frame_;
    std::string       udev_port_;
    int               baud_;

    double            frequency_;

    bool              publish_odom_;
    ros::Timer        periodicUpdateTimer_;

};



};



#endif