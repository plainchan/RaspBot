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
        sp_.close();
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
        catch(serial::IOException& e)
        {
            ROS_INFO_STREAM("failed to open port:"<< sp_.getPort());
            ROS_ERROR_STREAM(e.what());
            sp_.close();
            ros::requestShutdown();
            return false;
        }

        if(sp_.isOpen())
        {
            //echo port status
            ROS_INFO_STREAM("serial port opened succeed");
            //print port information
            ROS_INFO_STREAM("port:" << sp_.getPort());
            ROS_INFO_STREAM("baud:"<< sp_.getBaudrate());

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

        //read and process serial buff
        size_t buff_size = sp_.available();
        if(buff_size)
        {
            uint8_t RxBuff[MAX_BUFF_SIZE];

            if(buff_size>MAX_BUFF_SIZE)
                buff_size = MAX_BUFF_SIZE;

            buff_size = sp_.read(RxBuff,buff_size);
            for(int i=0;i<buff_size;++i)
            {

            }
        }

    }

    void BotBase::speedTwistCallBack(const geometry_msgs::Twist::ConstPtr &msg_ptr)
    {
        Frame_Speed_msgs frame;
        frame.header[0] = Header1;
        frame.header[1] = Header2;
        frame.len = 5;
        frame.crc = 0;
        frame.speed.data_tag = speed_tag;
        frame.speed.velocity = (int16_t)(msg_ptr->linear.x*1000);
        frame.speed.yaw =(int16_t)(msg_ptr->angular.z*1000);
        
        std::vector<uint8_t> Bytes = structPack_Bytes<Frame_Speed_msgs>(frame);
        // ROS_INFO("%x",Bytes[0]);
        // ROS_INFO("%x",Bytes[1]);
        // ROS_INFO("%d",(short)Bytes[7]<<8 | Bytes[6]);
        // ROS_INFO("%d",(short)Bytes[9]<<8 | Bytes[8]);
        sp_.write(Bytes);
 
    }

}