#include "raspbot_bringup/raspbot_base.h"
#include "raspbot_bringup/raspbot_comm.hpp"

namespace raspbot
{

    BotBase::BotBase(ros::NodeHandle nh,ros::NodeHandle private_nh):
    nh_(nh),
    nhPrivate_(private_nh),
    base_frame_("base_link"),
    odom_frame_("odom"),
    udev_port_("/dev/raspbot_com_port"),
    baud_(115200),
    frequency_(50)
    {

    }
    BotBase::~BotBase()
    {

    }
    
    void BotBase::initialize()
    {
        setting();

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data",10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("wheel_odom",10);


        periodicUpdateTimer_ = nh_.createTimer(ros::Duration(1./frequency_),&BotBase::periodicUpdate,this);

    }
    
    void BotBase::setting()
    {
        nhPrivate_.param<std::string>("base_frame",base_frame_,"base_link");
        nhPrivate_.param<std::string>("odom_frame",odom_frame_,"odom");
        nhPrivate_.param<std::string>("udev_port",udev_port_,"/dev/raspbot_com_port");
        nhPrivate_.param<int>("baud",baud_,115200);
        nhPrivate_.param<double>("frequency",frequency_,50);

        serial_init();

        twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel",10,&BotBase::speedTwistCallBack,this);

    }

    bool BotBase::serial_init()
    {
        sp_.setPort(udev_port_);
        sp_.setBaudrate(baud_);
        sp_.setBytesize(serial::eightbits);
        sp_.setParity(serial::parity_none);
        sp_.setStopbits(serial::stopbits_one);
        sp_.setFlowcontrol(serial::flowcontrol_none);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        sp_.setTimeout(timeout);

        try
        {
            sp_.open();
        }
        catch(serial::PortNotOpenedException& e)
        {
            ROS_INFO_STREAM("failed to open port");
            ROS_ERROR_STREAM(e.what());
            return false;
        }

        if(sp_.isOpen())
        {
            //print port information
            ROS_INFO_STREAM("serial port opened succeed");
 
            ROS_INFO_STREAM(sp_.getPort());
            ROS_INFO_STREAM(sp_.getBaudrate());

            return true;
        }
        else
        {
            ROS_INFO_STREAM("serial port opened failed");
            return false;
        }
    }
    void BotBase::periodicUpdate(const ros::TimerEvent &event)
    {

    }

    void BotBase::speedTwistCallBack(const geometry_msgs::Twist::ConstPtr &msg_ptr)
    {
        Speed_msgs speed;
        speed.header1 = Header1;
        speed.header2 = Header2;
        speed.data_tag = speed_tag;
        speed.length = frame_speed_len;
        speed.velocity = (int16_t)(msg_ptr->linear.x*1000);
        speed.yaw =(int16_t)(msg_ptr->angular.z*1000);
        
        std::vector<uint8_t> Bytes = structPack_Bytes<Speed_msgs>(speed);
        ROS_INFO("%x",Bytes[0]);
        ROS_INFO("%x",Bytes[1]);
        ROS_INFO("%d",(short)Bytes[5]<<8 | (short)Bytes[4]);
        ROS_INFO("%d",(short)Bytes[7]<<8 | (short)Bytes[6]);
        sp_.write(Bytes);
 
    }

}