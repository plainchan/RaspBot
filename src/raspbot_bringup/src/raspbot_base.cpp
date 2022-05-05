#include "raspbot_bringup/raspbot_base.h"

namespace raspbot
{

    BotBase::BotBase(ros::NodeHandle nh,ros::NodeHandle private_nh):
    nh_(nh),
    nhPrivate_(private_nh)
    {
        // twist_sub = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel",10,raspbotTwistCallBack);

        nhPrivate_.param<std::string>("base_frame",base_frame,"base_link");
        nhPrivate_.param<std::string>("odom_frame",odom_frame,"odom");
        nhPrivate_.param<std::string>("udev_port",udev_port,"raspbot_com_port");
        nhPrivate_.param<int>("baud",baud,115200);

        serial_init();
    }
    BotBase::~BotBase()
    {

    }

    int BotBase::serial_init()
    {
        sp.setPort(udev_port);
        sp.setBaudrate(baud);
        try
        {
            sp.open();
        }
        catch(serial::PortNotOpenedException& e)
        {
            ROS_ERROR_STREAM(e.what());
            return -1;
        }

        if(sp.isOpen())
        {
            ROS_INFO_STREAM("serial port opens succeed");
            return 0;
        }
        else
        {
            ROS_INFO_STREAM("unknown error");
            return -1;
        }
    }

    void BotBase::loop()
    {
        ros::Rate r(50);
        while(ros::ok)
        {
            r.sleep();
            ros::spinOnce;
        }
    }



}