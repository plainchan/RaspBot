/*
 * MIT License
 *
 * Copyright (c) 2022 plainchan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "cmath"
#include "raspbot_bringup/raspbot_base.h"
#include "raspbot_bringup/crc16.h"
#include "raspbot_bringup/crc8.h"
#include "raspbot_bringup/raspbot_params.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "raspbot_msgs/bot_speed.h"
#include "raspbot_msgs/bot_encoder_debug.h"
#include "dynamic_reconfigure/server.h"
#include "raspbot_bringup/pid_debugConfig.h"
#include "functional"


/**
 * rosparams:
 *  base_frame
 *  odom_frame
 *  imu_frame
 *  odom_topic
 *  imu_topic
 *  twist_topic
 *  wheel_path_topic
 *  udev_port
 *  baud
 *  frequency
 *  publish_odomTF
 *  publish_wheel_path
 */




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
          imu_topic_("imu/data"),
          odom_topic_("wheel_odom"),
          twist_topic_("cmd_vel"),
          speed_topic_("speed"),
          wheel_path_topic_("wheel_path"),
          encoder_debug_topic_("encoder_debug"),
          raw_wheel_pose_x_(0.0),
          raw_wheel_pose_y_(0.0),
          raw_wheel_pose_theta_(0.0),
          linearSpeed_(0.0),
          angularSpeed_(0.0),
          turnRadius_(0.0),
          publish_odomTF_(false),
          publish_wheel_path_(false),
          publish_speed_(false),
          publish_encoder_debug_(false)

          
    {
        robot_msgs.voltage = 11.1;
        robot_msgs.l_encoder_pulse = 0;
        robot_msgs.r_encoder_pulse = 0;
        robot_msgs.acc[0] = 0.0;
        robot_msgs.acc[1] = 0.0;
        robot_msgs.acc[2] = 0.0;
        robot_msgs.gyr[0] = 0.0;
        robot_msgs.gyr[1] = 0.0;
        robot_msgs.gyr[2] = 0.0;
#ifdef  imu_mag
        robot_msgs.mag[0] = 0.0;
        robot_msgs.mag[1] = 0.0;
        robot_msgs.mag[2] = 0.0;
#endif /* imu_mag */
        robot_msgs.elu[0] = 0.0;
        robot_msgs.elu[1] = 0.0;
        robot_msgs.elu[2] = 0.0;

        stream_msgs.len = 0;
        stream_msgs.crc = 0;

        for(int i=0;i<MAX_BUFF_SIZE;++i)
            stream_msgs.stream_buff[i]=0;


    }
    BotBase::~BotBase()
    {
        /* stop car */
        sendFrame_Speed_dpkg();

        sp_.close();
    }

    void BotBase::initialize()
    {
        setting();

        //sub
        twist_sub_ = nh_.subscribe<geometry_msgs::Twist>(twist_topic_, 10, &BotBase::speedTwistCallBack, this);
       
        //pub
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
        wheel_path_pub_ = nh_.advertise<nav_msgs::Path>(wheel_path_topic_,10);
        speed_pub_ = nh_.advertise<raspbot_msgs::bot_speed>(speed_topic_,10);
        encoder_debug_pub_ = nh_.advertise<raspbot_msgs::bot_encoder_debug>(encoder_debug_topic_,10);


        //Timer
        periodicUpdateTimer_ = nh_.createTimer(ros::Duration(1. / frequency_), &BotBase::periodicUpdate, this);
    }

    /* load params and subscribe topic  */
    void BotBase::setting()
    {
        nhPrivate_.param<std::string>("base_frame", base_frame_, "base_link");
        nhPrivate_.param<std::string>("odom_frame", odom_frame_, "odom");
        nhPrivate_.param<std::string>("imu_frame", imu_frame_, "imu_link");

        nhPrivate_.param<std::string>("odom_topic", odom_topic_, "wheel/odom");
        nhPrivate_.param<std::string>("imu_topic", imu_topic_, "imu/data");
        nhPrivate_.param<bool>("publish_odomTF", publish_odomTF_, true);

        nhPrivate_.param<std::string>("wheel_path_topic", wheel_path_topic_, "wheel/path");
        nhPrivate_.param<bool>("publish_wheel_path", publish_wheel_path_, false);


        nhPrivate_.param<std::string>("twist_topic", twist_topic_, "cmd_vel");

        nhPrivate_.param<std::string>("speed_topic", speed_topic_, "speed");
        nhPrivate_.param<bool>("publish_speed", publish_speed_, false);
        
        nhPrivate_.param<std::string>("encoder_debug_topic", encoder_debug_topic_, "encoder_debug");
        nhPrivate_.param<bool>("publish_encoder_debug", publish_encoder_debug_, false);

        nhPrivate_.param<std::string>("udev_port", udev_port_, "/dev/raspbot_com_port");
        nhPrivate_.param<int>("baud", baud_, 115200);
        
        nhPrivate_.param<double>("frequency", frequency_, 50);
        
        if(frequency_>1000 ||frequency_<0)
        {
            ROS_FATAL_STREAM("Inappropriate frequency");
            frequency_ = 50.0;
            ROS_WARN_STREAM("reset frequency" <<frequency_ <<"HZ" );
        }
        ROS_INFO_STREAM("Timer:" << 1000.0/frequency_<<" ms");

        serial_init();

        //dynamic_reconfigure
        //use rqt plugin to change P,I,D
        //Ang change will triger action which send pid to mcu by serial
        //for debugging pid params conveniently
        //use rqt_plot to check encoder pluse whether is suitable or not

        // dynamic_pid_callback_ = boost::bind(&BotBase::dynamicPIDCallback,this,_1,_2);  // same as std::bind
        dynamic_pid_callback_ = std::bind(&BotBase::dynamicPIDCallback,this,std::placeholders::_1,std::placeholders::_2);
        dynamic_pid_server_.setCallback(dynamic_pid_callback_);

    }

    bool BotBase::serial_init()
    {
        sp_.setPort(udev_port_);
        sp_.setBaudrate(baud_);
        sp_.setBytesize(serial::eightbits);
        sp_.setParity(serial::parity_none);
        sp_.setStopbits(serial::stopbits_one);
        sp_.setFlowcontrol(serial::flowcontrol_none);

        /**
         * @brief brief 读取或写入数据之后，延时一段时间,以使读取完成或发送完成
         * @note  接收时未使用timeout,使用定时器周期读取串口，使用available()检测串口是否有数据，有数据则处理，cpu数据很快
         *        发送时使用timeout,发送是订阅cmd_vel话题,由于cmd_vel话题频率通常小于50HZ，所以也可不用设置
         * @bug     
         * @param inter_byte_timeout serial::Timeout::max()  禁止读取每个字节时延时
         *        read_timeout_constant      0       buff读取完成后不延时
         *        read_timeout_multiplier    0       buff读取完成后，不延时读取的Bytes个时间
         *        write_timeout_constant     0       buff写入完成后不延时
         *        write_timeout_multiplier   1       buff读取完成后，延时写入的Bytes个时间(根据写入时间延迟*因子)[Recommand]
         */
        serial::Timeout timeout(serial::Timeout::max(),0,0,2,0); //发送后延时2ms
        sp_.setTimeout(timeout);

        try
        {
            sp_.open();

            /* echo port status */
            ROS_INFO_STREAM("serial port opened succeed");
            /* print port information */
            ROS_INFO_STREAM("port:" << sp_.getPort());
            ROS_INFO_STREAM("baud:" << sp_.getBaudrate());
            
            /* clear cache  */
            if(sp_.available())
                sp_.read(sp_.available());

            return true;
        }
        catch (serial::IOException &e)
        {
            /**
             * @brief 捕获端口异常，告知错误点，然后重新尝试打开，会终止程序，配合ROS launch的respawn
             *        可重复启动进程，以使其USB重新插拔后，自动检测端口，无需手动再次启动节点
             */
            ROS_ERROR_STREAM("failed to open port:" << sp_.getPort());
            sp_.open();
            return false;
        }
        ROS_INFO("Here");
        
    }

    void BotBase::periodicUpdate(const ros::TimerEvent &event)
    {

        // ROS_INFO_STREAM("run time:" << event.profile.last_duration  << "\n"
        //                 "last_expected:" <<event.last_expected   << "\n"
        //                 "current_expected:" <<event.current_expected << "\n"
        //                 "last_real:" <<event.last_real << "\n"
        //                 "current_real:" <<event.current_real << "\n"
        //                 "last_expired:" <<event.last_expired << "\n"
        //                 "current_expired:" <<event.current_expired << "\n"
        // );
        // read and process serial buff
        size_t buff_size = sp_.available();
        if (buff_size)
        {
            uint8_t RxBuff[MAX_RxBUFF_SIZE];

            // 缓冲区太大，选择最近的数据
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
    
       
            for (int i = 0; i < buff_size; ++i)
            {
                if (parse_stream(stream_msgs, RxBuff[i]) == 1)
                {   
                    if(imu_updated)
                        publishIMU();
                    if(encoder_updated)
                    {
                        if(publish_odomTF_) publishTransformAndOdom();
                        if(publish_speed_)  publishSpeed();
                    }

                    imu_updated = false;
                    encoder_updated = false;
                    

                    /*  show params */
// #define debug_robot_params
#ifdef  debug_robot_params
                    static int count=0;
                    if(++count>frequency_)
                    {
                        ROS_INFO_STREAM("------------------------------------\n"<<
                        "V       "      << robot_msgs.voltage << "\n" <<     
                        "encoder "      << robot_msgs.l_encoder_pulse <<"\t"
                                        << robot_msgs.r_encoder_pulse   <<"\n"
                        "acc     "      << robot_msgs.acc[0] <<"\t"
                                        << robot_msgs.acc[1] <<"\t"
                                        << robot_msgs.acc[2] <<"\n"  
                        "gyr     "      << robot_msgs.gyr[0] <<"\t"
                                        << robot_msgs.gyr[1] <<"\t"
                                        << robot_msgs.gyr[2] <<"\n"
                        #ifdef imu_mag
                        "mag     "      << robot_msgs.mag[0] <<"\t"
                                        << robot_msgs.mag[1] <<"\t"
                                        << robot_msgs.mag[2] <<"\n"
                        #endif
                        "elu     "      << robot_msgs.elu[0] <<"\t"
                                        << robot_msgs.elu[1] <<"\t"
                                        << robot_msgs.elu[2] 
                        );
                        count=0;
                    }
                    
#endif

                }
            }

        }

    }
    int BotBase::parse_stream(Stream_msgs &stream_msgs, const uint8_t buff)
    {
        static uint16_t bytesCount=0;
        stream_msgs.stream_buff[bytesCount++] = buff;
        if (bytesCount == FRAME_HEAD_OFFSET) //检查帧头
        {
            if (stream_msgs.stream_buff[0] != Header1 || stream_msgs.stream_buff[1] != Header2)
            {
                bytesCount = 1;
                stream_msgs.stream_buff[0] = stream_msgs.stream_buff[1];
                return -1; //错误帧
            }
        }
        else if (bytesCount == FRAME_DPKG_LEN_OFFSET) // DPKG 长度
        {
            stream_msgs.len = stream_msgs.stream_buff[bytesCount - 1];
            if (stream_msgs.len > MAX_DPKG_SIZE)
            {
                bytesCount = 0;
                return -3;          //DPKG 长度错误
            }
        }
        else if (bytesCount == FRAME_HEAD_CRC_OFFSET) // crc 帧头校验
        {
            stream_msgs.crc = stream_msgs.stream_buff[bytesCount - 1]; //crc8
            // stream_msgs.crc = Byte2U16(&stream_msgs.stream_buff[bytesCount - 2]); //crc16
            if(stream_msgs.crc!=crc_8(stream_msgs.stream_buff,FRAME_CALCU_CRC_BYTES))
            {
                ROS_WARN_STREAM("Erro of header's CRC");
                bytesCount = 0;
                return -1; //校验错误
            }

        }
        else if (bytesCount >= FRAME_INFO_SIZE + stream_msgs.len+FRAME_DPKG_CRC_BYTES) 
        {
            
            stream_msgs.crc = Byte2U16(&stream_msgs.stream_buff[bytesCount-FRAME_DPKG_CRC_BYTES]);
            bytesCount = 0;
            if(stream_msgs.crc!=crc_16(&stream_msgs.stream_buff[FRAME_INFO_SIZE],stream_msgs.len))
            {
                ROS_WARN_STREAM("Erro of data's CRC");
                return -1; //校验错误
            }
            return decode_frame(stream_msgs);
        }

        return 0; //帧未就绪
    }

    int BotBase::decode_frame(Stream_msgs &stream_msgs)
    {
        uint8_t offset = 0;
        const uint8_t *buff = &stream_msgs.stream_buff[FRAME_INFO_SIZE];
        uint8_t len = stream_msgs.len;
        while(offset < len)
        {
            switch (buff[offset])
            {
            case robot_tag:

                robot_msgs.voltage = ((float)buff[++offset]) / 10.0;

                robot_msgs.l_encoder_pulse = Byte2INT16(&buff[offset+1]);offset+=2;
                robot_msgs.r_encoder_pulse = Byte2INT16(&buff[offset+1]);offset+=2; 
                encoder_updated = true;

                robot_msgs.acc[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.acc[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.acc[2] = Byte2Float(&buff[offset+1]);offset+=4;

                robot_msgs.gyr[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.gyr[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.gyr[2] = Byte2Float(&buff[offset+1]);offset+=4;
        #ifdef  imu_mag
                robot_msgs.mag[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.mag[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.mag[2] = Byte2Float(&buff[offset+1]);offset+=4;
        #endif          
                robot_msgs.elu[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.elu[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.elu[2] = Byte2Float(&buff[offset+1]);offset+=4;
                imu_updated = true;
                break; /* robot_tag */
            case voltage_tag:
                robot_msgs.voltage = ((float)buff[++offset])/10.0;
                break; /* voltage_tag */
            case encoder_tag:
                robot_msgs.l_encoder_pulse = Byte2INT16(&buff[offset+1]);offset+=2;
                robot_msgs.r_encoder_pulse = Byte2INT16(&buff[offset+1]);offset+=2;
                encoder_updated = true;
                break; /* encoder_tag */
            case imu_tag:
                robot_msgs.acc[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.acc[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.acc[2] = Byte2Float(&buff[offset+1]);offset+=4;

                robot_msgs.gyr[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.gyr[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.gyr[2] = Byte2Float(&buff[offset+1]);offset+=4;
        #ifdef  imu_mag
                robot_msgs.mag[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.mag[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.mag[2] = Byte2Float(&buff[offset+1]);offset+=4;
        #endif          
                robot_msgs.elu[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.elu[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.elu[2] = Byte2Float(&buff[offset+1]);offset+=4;
                imu_updated = true;
                break; /* imu_tag */
            case imu_sensor_tag:
                robot_msgs.acc[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.acc[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.acc[2] = Byte2Float(&buff[offset+1]);offset+=4;

                robot_msgs.gyr[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.gyr[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.gyr[2] = Byte2Float(&buff[offset+1]);offset+=4;
        #ifdef imu_mag
                robot_msgs.mag[0] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.mag[1] = Byte2Float(&buff[offset+1]);offset+=4;
                robot_msgs.mag[2] = Byte2Float(&buff[offset+1]);offset+=4;
        #endif  
                imu_updated = true;
                break; /* imu_sensor_tag */
            case imu_raw_tag:
                robot_msgs.acc[0] = Byte2INT16(&buff[offset+1])*accRatio;offset+=2;
                robot_msgs.acc[1] = Byte2INT16(&buff[offset+1])*accRatio;offset+=2;
                robot_msgs.acc[2] = Byte2INT16(&buff[offset+1])*accRatio;offset+=2;

                robot_msgs.gyr[0] = Byte2INT16(&buff[offset+1])*gyrRatio;offset+=2;
                robot_msgs.gyr[1] = Byte2INT16(&buff[offset+1])*gyrRatio;offset+=2;
                robot_msgs.gyr[2] = Byte2INT16(&buff[offset+1])*gyrRatio;offset+=2;
        #ifdef  imu_mag
                robot_msgs.mag[0] = Byte2INT16(&buff[offset+1])*magRatio;offset+=2;
                robot_msgs.mag[1] = Byte2INT16(&buff[offset+1])*magRatio;offset+=2;
                robot_msgs.mag[2] = Byte2INT16(&buff[offset+1])*magRatio;offset+=2;
        #endif          
                robot_msgs.elu[0] = Byte2INT16(&buff[offset+1])*eluRatio;offset+=2;
                robot_msgs.elu[1] = Byte2INT16(&buff[offset+1])*eluRatio;offset+=2;
                robot_msgs.elu[2] = Byte2INT16(&buff[offset+1])*eluRatio;offset+=2;
                imu_updated = true;
                break; /* imu_raw_tag */

            default:
                ++offset;
                break; /* default */
            }
            ++offset;
        }

        return 1;
    }
    void BotBase::speedTwistCallBack(const geometry_msgs::Twist::ConstPtr &msg_ptr)
    {
        if(!sendFrame_Speed_dpkg(msg_ptr->linear.x,msg_ptr->angular.z))
        {
            ROS_WARN_STREAM("Serial send failed");
        }
        if(publish_encoder_debug_)
            publishEncoderDebug(msg_ptr->linear.x,msg_ptr->angular.z);
    }

    void BotBase::dynamicPIDCallback(dynamic_pid::pid_debugConfig &config,uint32_t level)
    {
        //print pid info
        ROS_INFO_STREAM("P:"<< config.P <<"\tI:" << config.I <<"\tD:"<< config.D);
        sendFrame_PID_dpkg(config.P,config.I,config.D);

    }

    void BotBase::publishIMU()
    {
        imu_.header.frame_id = imu_frame_;
        imu_.header.seq = 100;
        imu_.header.stamp = ros::Time::now();

        imu_.linear_acceleration.x = robot_msgs.acc[0];
        imu_.linear_acceleration.y = robot_msgs.acc[1];
        imu_.linear_acceleration.z = robot_msgs.acc[2];

        imu_.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        imu_.angular_velocity.x = robot_msgs.gyr[0];
        imu_.angular_velocity.y = robot_msgs.gyr[1];
        imu_.angular_velocity.z = robot_msgs.gyr[2];

        imu_.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        /* 四元数 */
        tf2::Quaternion qtn;
        static double roll_offset = robot_msgs.elu[0];
        static double pitch_offset = robot_msgs.elu[1];
        static double yaw_offset = robot_msgs.elu[2];

        qtn.setRPY(robot_msgs.elu[0]-roll_offset,robot_msgs.elu[1]-pitch_offset,robot_msgs.elu[2]-yaw_offset);
   
        imu_.orientation.w = qtn.getW();
        imu_.orientation.x = qtn.getX();
        imu_.orientation.y = qtn.getY();
        imu_.orientation.z = qtn.getZ();

        imu_.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        //publish
        imu_pub_.publish(imu_);
    }

    void BotBase::publishTransformAndOdom()
    {

        double l_motor_speed = 2*M_PI*wheelRadius*robot_msgs.l_encoder_pulse/(PPR*intervalTimer);
        double r_motor_speed = 2*M_PI*wheelRadius*robot_msgs.r_encoder_pulse/(PPR*intervalTimer);

        linearSpeed_ = (l_motor_speed+r_motor_speed)/2;
        angularSpeed_ = (r_motor_speed-l_motor_speed)/wheelTrack;
        
        turnRadius_ = linearSpeed_/angularSpeed_;


        // raw_wheel_pose_theta = raw_wheel_pose_theta + angularSpeed*intervalTimer;  //?
        raw_wheel_pose_x_ = raw_wheel_pose_x_ + linearSpeed_*intervalTimer*cos(raw_wheel_pose_theta_); 
        raw_wheel_pose_y_ = raw_wheel_pose_y_ + linearSpeed_*intervalTimer*sin(raw_wheel_pose_theta_);
        raw_wheel_pose_theta_ = raw_wheel_pose_theta_ + angularSpeed_*intervalTimer; //?

       
        tf2::Quaternion qtn;
        qtn.setRPY(0,0,raw_wheel_pose_theta_);

        /* publish transform  */
        odomtfs_.header.frame_id = odom_frame_;
        odomtfs_.header.seq = 10;
        odomtfs_.header.stamp = ros::Time::now();

        odomtfs_.child_frame_id = base_frame_;
        
        odomtfs_.transform.translation.x = raw_wheel_pose_x_;
        odomtfs_.transform.translation.y = raw_wheel_pose_y_;
        odomtfs_.transform.translation.z = 0.0;

        odomtfs_.transform.rotation.w = qtn.getW();
        odomtfs_.transform.rotation.x = qtn.getX();
        odomtfs_.transform.rotation.y = qtn.getY();
        odomtfs_.transform.rotation.z = qtn.getZ();

        tfBroadcaster_.sendTransform(odomtfs_);  //publsih

  
        /*  publish odom  */
        wheel_odom_.header.frame_id = odom_frame_;
        wheel_odom_.header.seq = 50;
        wheel_odom_.header.stamp = ros::Time::now();

        wheel_odom_.child_frame_id = base_frame_;

        /**  The pose in this message should be specified 
         *  in the coordinate frame given by header.frame_id. 
         */
        wheel_odom_.pose.pose.position.x = raw_wheel_pose_x_;
        wheel_odom_.pose.pose.position.y = raw_wheel_pose_y_;
        wheel_odom_.pose.pose.position.z = 0.0;
        wheel_odom_.pose.pose.orientation.w = qtn.getW();
        wheel_odom_.pose.pose.orientation.x = qtn.getX();
        wheel_odom_.pose.pose.orientation.y = qtn.getY();
        wheel_odom_.pose.pose.orientation.z = qtn.getZ();    //the position  and orientation of the car in the odom frame
        wheel_odom_.pose.covariance={0};    

        /**
         * The twist in this message should be specified 
         * in the coordinate frame given by the child_frame_id
         */  
        wheel_odom_.twist.twist.linear.x=linearSpeed_;
        wheel_odom_.twist.twist.linear.y=0;
        wheel_odom_.twist.twist.linear.z=0;
        wheel_odom_.twist.twist.angular.x=0;
        wheel_odom_.twist.twist.angular.y=0;
        wheel_odom_.twist.twist.angular.z= angularSpeed_;      //the speed  and orientation of the car in the car frame

        wheel_odom_.twist.covariance={0};
        
        odom_pub_.publish(wheel_odom_);


        /*  publish wheel path  */
        if(publish_wheel_path_)
        {
            static bool initHeader = true;
            if(initHeader)
            {
                wheel_path_.header.frame_id = odom_frame_;
                wheel_path_.header.seq = 20;
                wheel_path_.header.stamp = ros::Time::now();
                initHeader = false;
            }
            geometry_msgs::PoseStamped cur_pose;
            cur_pose.header.frame_id = odom_frame_;
            cur_pose.header.seq = 0;
            cur_pose.header.stamp = ros::Time::now();

            cur_pose.pose.position.x = raw_wheel_pose_x_;
            cur_pose.pose.position.y = raw_wheel_pose_y_;
            cur_pose.pose.position.z = 0.0;

            cur_pose.pose.orientation.w = qtn.getW();
            cur_pose.pose.orientation.x = qtn.getX();
            cur_pose.pose.orientation.y = qtn.getY();
            cur_pose.pose.orientation.z = qtn.getZ();

            wheel_path_.poses.emplace_back(cur_pose);

            wheel_path_pub_.publish(wheel_path_);
        }
  
    }
    void BotBase::publishSpeed()
    {
        speed_.linear = linearSpeed_;
        speed_.anguar = angularSpeed_;
        speed_.radius = turnRadius_;

        speed_pub_.publish(speed_);
    }

    void BotBase::publishEncoderDebug(double linear,double angular)
    {

        encoder_debug_.encoder_l = robot_msgs.l_encoder_pulse;
        encoder_debug_.encoder_r = robot_msgs.r_encoder_pulse;

	    float L_ideal_velocity = linear - angular * wheelTrack / 2;
	    float R_ideal_velocity = linear + angular * wheelTrack / 2;

        encoder_debug_.encoder_ideal_l = L_ideal_velocity * intervalTimer * PPR / (2 * wheelRadius * M_PI); 
        encoder_debug_.encoder_ideal_r = R_ideal_velocity * intervalTimer * PPR / (2 * wheelRadius * M_PI); 
        
        encoder_debug_pub_.publish(encoder_debug_);
    }


    bool BotBase::sendFrame_Speed_dpkg(double speed,double angular)
    {
        
        Frame_Speed_dpkg frame;

        frame.header[0] = Header1;
        frame.header[1] = Header2;
        frame.len = speed_dpkg_len;
        frame.crc_head = 0;
        
        frame.speed.data_tag = speed_tag;
        frame.speed.velocity = speed*1000;
        frame.speed.angular = angular*1000;
        frame.crc_dpkg= 0;

        std::vector<uint8_t> Bytes = structPack_Bytes<Frame_Speed_dpkg>(frame);

        uint8_t CRC8  = crc_8(Bytes.data(),FRAME_CALCU_CRC_BYTES);
        uint16_t CRC16 = crc_16(Bytes.data()+FRAME_INFO_SIZE,speed_dpkg_len);

        /* reset crc value  */
        setBuffCRCValue(Bytes,FRAME_DPKG_LEN_OFFSET,CRC8);
        setBuffCRCValue<uint16_t,2>(Bytes,FRAME_INFO_SIZE+speed_dpkg_len,CRC16);

        // debug
        // ROS_INFO("Header1:%x", Bytes[0]);      
        // ROS_INFO("Header2:%x", Bytes[1]);    
        // ROS_INFO("Len:%d", Bytes[2]);  
        // ROS_INFO("crc:%d", Bytes[3]); 
        // ROS_INFO("tag:%d", Bytes[4]); 
        // ROS_INFO("velocity:%.2f",Byte2INT16(&Bytes[5])/1000.0);
        // ROS_INFO("angular:%.2f",Byte2INT16(&Bytes[7])/1000.0);
        // ROS_INFO("crc:%d",Byte2U16(&Bytes[9]));    


        size_t send_count=sp_.write(Bytes);
        return send_count==Bytes.size();
    }
    bool BotBase::sendFrame_PID_dpkg(float p,float i,float d)
    {
        
        Frame_PID_dpkg frame;

        frame.header[0] = Header1;
        frame.header[1] = Header2;
        frame.len = pid_dpkg_len;
        frame.crc_head = 0;
        
        frame.pid.data_tag = pid_tag;
        frame.pid.P = p*10;
        frame.pid.I = i*10;
        frame.pid.D = d*10;
        frame.crc_dpkg= 0;

        std::vector<uint8_t> Bytes = structPack_Bytes<Frame_PID_dpkg>(frame);

        uint8_t CRC8  = crc_8(Bytes.data(),FRAME_CALCU_CRC_BYTES);
        uint16_t CRC16 = crc_16(Bytes.data()+FRAME_INFO_SIZE,pid_dpkg_len);

        /* reset crc value  */
        setBuffCRCValue(Bytes,FRAME_DPKG_LEN_OFFSET,CRC8);
        setBuffCRCValue<uint16_t,2>(Bytes,FRAME_INFO_SIZE+pid_dpkg_len,CRC16);

        // debug
        // ROS_INFO("Header1:%x", Bytes[0]);      
        // ROS_INFO("Header2:%x", Bytes[1]);    
        // ROS_INFO("Len:%d", Bytes[2]);  
        // ROS_INFO("crc:%d", Bytes[3]); 
        // ROS_INFO("tag:%d", Bytes[4]); 
        // ROS_INFO("P:%.2f",Byte2INT16(&Bytes[5])/10.0);
        // ROS_INFO("I:%.2f",Byte2INT16(&Bytes[7])/10.0);
        // ROS_INFO("D:%.2f",Byte2INT16(&Bytes[9])/10.0);
        // ROS_INFO("crc:%d",Byte2U16(&Bytes[11]));    

        size_t send_count=sp_.write(Bytes);
        return send_count==Bytes.size();
    }
}