#include "raspbot_bringup/raspbot_base.h"

namespace raspbot
{

    BotBase::BotBase(ros::NodeHandle nh, ros::NodeHandle private_nh)
        : nh_(nh),
          nhPrivate_(private_nh),
          base_frame_("base_link"),
          odom_frame_("odom"),
          imu_frame_("imu_link"),
          udev_port_("/dev/raspbot_com_port"),
          baud_(115200),
          frequency_(50),
          publish_odom_(true),
          imu_topic_("imu/data"),
          odom_topic_("wheel_odom")
    {
//         robot_msgs.crc = 0;
//         robot_msgs.len = 0;
//         robot_msgs.robot_msgs.voltage = 0;
//         robot_msgs.robot_msgs.l_encoder_pulse = 0;
//         robot_msgs.robot_msgs.r_encoder_pulse = 0;
//         robot_msgs.robot_msgs.acc[0] = 0.0;
//         robot_msgs.robot_msgs.acc[1] = 0.0;
//         robot_msgs.robot_msgs.acc[2] = 0.0;
//         robot_msgs.robot_msgs.gyr[0] = 0.0;
//         robot_msgs.robot_msgs.gyr[1] = 0.0;
//         robot_msgs.robot_msgs.gyr[2] = 0.0;
// #ifdef  imu_mag
//         robot_msgs.robot_msgs.mag[0] = 0.0;
//         robot_msgs.robot_msgs.mag[1] = 0.0;
//         robot_msgs.robot_msgs.mag[2] = 0.0;
// #endif
        // ROS_INFO_STREAM((uint16_t)robot_msgs.len);
        // ROS_INFO_STREAM(robot_msgs.crc);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.voltage);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.l_encoder_pulse);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.r_encoder_pulse);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.acc[0]);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.acc[1]);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.acc[2]);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.gyr[0]);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.gyr[1]);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.gyr[2]);
#ifdef imu_tag
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.mag[0]);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.mag[1]);
        // ROS_INFO_STREAM(robot_msgs.robot_msgs.mag[2]);
#endif
    }
    BotBase::~BotBase()
    {
        Frame_Speed_dpkg frame;
        frame.header[0] = Header1;
        frame.header[1] = Header2;
        frame.len = 5;
        frame.crc = 0;
        frame.speed.data_tag = speed_tag;
        frame.speed.velocity = 0;
        frame.speed.yaw = 0;
        std::vector<uint8_t> Bytes = structPack_Bytes<Frame_Speed_dpkg>(frame);
        sp_.write(Bytes);

        sp_.close();
    }

    void BotBase::initialize()
    {
        setting();

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);

        periodicUpdateTimer_ = nh_.createTimer(ros::Duration(1. / frequency_), &BotBase::periodicUpdate, this);
    }

    void BotBase::setting()
    {
        nhPrivate_.param<std::string>("base_frame", base_frame_, "base_link");
        nhPrivate_.param<std::string>("odom_frame", odom_frame_, "odom");
        nhPrivate_.param<std::string>("imu_frame", imu_frame_, "imu_link");

        nhPrivate_.param<std::string>("odom_topic", odom_topic_, "odom");
        nhPrivate_.param<std::string>("imu_topic", imu_topic_, "imu/data");
        nhPrivate_.param<bool>("publish_odom", publish_odom_, true);

        nhPrivate_.param<std::string>("twist_topic", twist_topic_, "cmd_vel");
        

        nhPrivate_.param<std::string>("udev_port", udev_port_, "/dev/raspbot_com_port");
        nhPrivate_.param<int>("baud", baud_, 115200);
        
        nhPrivate_.param<double>("frequency", frequency_, 50);

        serial_init();

        twist_sub_ = nh_.subscribe<geometry_msgs::Twist>(twist_topic_, 10, &BotBase::speedTwistCallBack, this);
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
        catch (serial::IOException &e)
        {
            ROS_INFO_STREAM("failed to open port:" << sp_.getPort());
            ROS_ERROR_STREAM(e.what());
            sp_.close();
            ros::requestShutdown();
            return false;
        }

        if (sp_.isOpen())
        {
            // echo port status
            ROS_INFO_STREAM("serial port opened succeed");
            // print port information
            ROS_INFO_STREAM("port:" << sp_.getPort());
            ROS_INFO_STREAM("baud:" << sp_.getBaudrate());

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

        // read and process serial buff
        size_t buff_size = sp_.available();
        if (buff_size)
        {
            uint8_t RxBuff[MAX_RxBUFF_SIZE];

            //缓冲区太大，选择最近的数据
            if (buff_size > MAX_RxBUFF_SIZE)
            {
                ROS_WARN_STREAM("Buffer overflow");
                if (buff_size > MAX_RxBUFF_SIZE + MAX_BUFF_SIZE)
                {
                    buff_size = buff_size - MAX_BUFF_SIZE;
                    uint8_t tempRxBuff[buff_size];
                    sp_.read(tempRxBuff, buff_size);
                    buff_size = MAX_BUFF_SIZE;
                }
                else
                {
                    buff_size = buff_size - MAX_BUFF_SIZE;
                    sp_.read(RxBuff, buff_size);
                    buff_size = MAX_BUFF_SIZE;
                }
            }

            buff_size = sp_.read(RxBuff, buff_size);
            // ROS_INFO_STREAM("time here");
            for (int i = 0; i < buff_size; ++i)
            {
                if (parse_stream(stream_msgs, RxBuff[i]) == 1)
                {   
                    setImuValue(imu_);
                    imu_pub_.publish(imu_);
                    if(publish_odom_)
                    {
                        // calcuOdomValue(wheel_odom_);
                        // odom_pub_.publish(wheel_odom_);
                    }
                }
                // else
                //     ROS_INFO_STREAM("parse here");
            }
        }


    }
    int BotBase::parse_stream(Stream_msgs &stream_msgs, const uint8_t buff)
    {
        static uint16_t bytesCount=0;
        stream_msgs.stream_buff[bytesCount++] = buff;

        ROS_INFO("%d",bytesCount);
        if (bytesCount == 2) //检查帧头
        {
            if (stream_msgs.stream_buff[0] != Header1 || stream_msgs.stream_buff[1] != Header2)
            {
                bytesCount = 1;
                stream_msgs.stream_buff[0] = stream_msgs.stream_buff[1];
                return -1; //错误帧
            }
        }
        else if (bytesCount == 3) // DPKG 长度
        {
            stream_msgs.len = stream_msgs.stream_buff[bytesCount - 1];
            if (stream_msgs.len > MAX_DPKG_SIZE)
            {
                bytesCount = 0;
                return -2;
            }
        }
        else if (bytesCount == 5) // crc
        {
            stream_msgs.crc = Bytes2Num<uint16_t,2>(&stream_msgs.stream_buff[bytesCount - 2]);
        }
        else if (bytesCount>3 && bytesCount >= stream_msgs.len + FRAME_INFO_SIZE)
        {
            bytesCount = 0;

            return decode_frame(stream_msgs);
        }

        return 0; //帧未就绪
    }

    int BotBase::decode_frame(Stream_msgs &stream_msgs)
    {
        uint8_t offset = 0;
        uint8_t *buff = &stream_msgs.stream_buff[FRAME_INFO_SIZE];
        switch (buff[offset])
        {
        case robot_tag:

            robot_msgs.voltage = ((float)buff[++offset]) / 10.0;
            
            robot_msgs.l_encoder_pulse = Bytes2Num<int16_t,2>(&buff[offset+1]);offset+=2;
            
            robot_msgs.r_encoder_pulse = Bytes2Num<int16_t,2>(&buff[offset+1]);offset+=2;
      
            robot_msgs.acc[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.acc[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.acc[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;

            robot_msgs.gyr[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.gyr[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.gyr[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
#ifdef  imu_mag
            robot_msgs.mag[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.mag[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.mag[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
#endif          
            robot_msgs.elu[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.elu[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.elu[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            break;

        case encoder_tag:
            robot_msgs.l_encoder_pulse = Bytes2Num<int16_t,2>(&buff[offset+1]);offset+=2;
            robot_msgs.r_encoder_pulse = Bytes2Num<int16_t,2>(&buff[offset+1]);offset+=2;
        case imu_tag:
            robot_msgs.acc[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.acc[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.acc[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;

            robot_msgs.gyr[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.gyr[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.gyr[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
#ifdef  imu_mag
            robot_msgs.mag[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.mag[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.mag[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
#endif          
            robot_msgs.elu[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.elu[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.elu[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
        case imu_sensor_tag:
            robot_msgs.acc[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.acc[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.acc[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;

            robot_msgs.gyr[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.gyr[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.gyr[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
#ifdef imu_mag
            robot_msgs.mag[0] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.mag[1] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
            robot_msgs.mag[2] = Bytes2Num<float,4>(&buff[offset+1]);offset+=4;
#endif
        default:
            ++offset;
            break;
        }

        return 1;
    }
    void BotBase::speedTwistCallBack(const geometry_msgs::Twist::ConstPtr &msg_ptr)
    {
        Frame_Speed_dpkg frame;
        frame.header[0] = Header1;
        frame.header[1] = Header2;
        frame.len = speed_dpkg_len;
        frame.crc = 219;
        frame.speed.data_tag = speed_tag;
        frame.speed.velocity = (int16_t)(msg_ptr->linear.x * 1000);
        frame.speed.yaw = (int16_t)(msg_ptr->angular.z * 1000);

        std::vector<uint8_t> Bytes = structPack_Bytes<Frame_Speed_dpkg>(frame);

#ifdef debug
        ROS_INFO("%x", Bytes[0]);
        ROS_INFO("%x", Bytes[1]);
        ROS_INFO("%d", Bytes[2]);
        ROS_INFO("%d", Bytes2Num<uint16_t,2>(&Bytes[3]));
        ROS_INFO("%x", Bytes[5]);
        ROS_INFO("%d", Bytes2Num<uint16_t,2>(&Bytes[6]));
        ROS_INFO("%d", Bytes2Num<uint16_t,2>(&Bytes[8]));
#endif

        sp_.write(Bytes);
    }

    void BotBase::setImuValue(sensor_msgs::Imu &imu)
    {
        imu.header.frame_id = imu_frame_;
        imu.header.seq = 100;
        imu.header.stamp = ros::Time::now();

        imu.linear_acceleration.x = robot_msgs.acc[0];
        imu.linear_acceleration.y = robot_msgs.acc[1];
        imu.linear_acceleration.z = robot_msgs.acc[2];

        imu.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        imu.angular_velocity.x = robot_msgs.gyr[0];
        imu.angular_velocity.y = robot_msgs.gyr[1];
        imu.angular_velocity.z = robot_msgs.gyr[2];

        imu.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        imu.orientation.w = 0;
        imu.orientation.x = 0;
        imu.orientation.y = 0;
        imu.orientation.z = 0;

        imu.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    }
    void BotBase::calcuOdomValue(nav_msgs::Odometry &odom)
    {
        // odom.child_frame_id=;
        // odom.header=;
        // odom.twist=;
        // odom.pose=;
    }
}