#include "raspbot_bringup/raspbot_base.h"
#include "raspbot_bringup/crc16.h"
#include "raspbot_bringup/crc8.h"
#include <algorithm>

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
        robot_msgs.voltage = 12.6;
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

        /* stop car  */
        Frame_Speed_dpkg frame;
        frame.header[0] = Header1;
        frame.header[1] = Header2;
        frame.len = 5;
        frame.crc_head = 0;
        frame.speed.data_tag = speed_tag;
        frame.speed.velocity = 0;
        frame.speed.yaw = 0;
        std::vector<uint8_t> Bytes = structPack_Bytes<Frame_Speed_dpkg>(frame);
        sp_.write(Bytes);

        /* stop car */
        sendFrame_Speed_dpkg();

        sp_.close();
    }

    void BotBase::initialize()
    {
        setting();

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);

        periodicUpdateTimer_ = nh_.createTimer(ros::Duration(1. / frequency_), &BotBase::periodicUpdate, this);
    }

    /* load params and subscribe topic  */
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
        
        if(frequency_>1000 ||frequency_<0)
        {
            ROS_FATAL_STREAM("Inappropriate frequency");
            frequency_ = 50.0;
            ROS_WARN_STREAM("reset frequency" <<frequency_ <<"HZ" );
        }
        ROS_INFO_STREAM("Timer:" << 1000.0/frequency_<<" ms");
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
        serial::Timeout timeout(serial::Timeout::max(),0,0,2,0); //发送后延时1ms
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
                    setImuValue(imu_);
                    imu_pub_.publish(imu_);
                    if(publish_odom_)
                    {
                        // calcuOdomValue(wheel_odom_);
                        // odom_pub_.publish(wheel_odom_);
                    }

                    /*  show params */
#define debug_robot_params
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
            // stream_msgs.crc = Bytes2Num<uint16_t,2>(&stream_msgs.stream_buff[bytesCount - 2]); //crc16
            if(stream_msgs.crc!=crc_8(stream_msgs.stream_buff,FRAME_CALCU_CRC_BYTES))
            {
                ROS_WARN_STREAM("Erro of header's CRC");
                bytesCount = 0;
                return -1; //校验错误
            }

        }
        else if (bytesCount >= FRAME_INFO_SIZE + stream_msgs.len+FRAME_DPKG_CRC_BYTES) 
        {
            
            stream_msgs.crc = Bytes2Num<uint16_t,2>(&stream_msgs.stream_buff[bytesCount-FRAME_DPKG_CRC_BYTES]);
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
        uint8_t *buff = &stream_msgs.stream_buff[FRAME_INFO_SIZE];
        uint8_t len = stream_msgs.len;
        while(offset < len)
        {
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
                break; /* robot_tag */
            case voltage_tag:
                robot_msgs.voltage = ((float)buff[++offset])/10.0;
                break; /* voltage_tag */
            case encoder_tag:
                robot_msgs.l_encoder_pulse = Bytes2Num<int16_t,2>(&buff[offset+1]);offset+=2;
                robot_msgs.r_encoder_pulse = Bytes2Num<int16_t,2>(&buff[offset+1]);offset+=2;
                break; /* encoder_tag */
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
                break; /* imu_tag */
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
                break; /* imu_sensor_tag */
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

    bool BotBase::sendFrame_Speed_dpkg(float speed,float yaw)
    {
        
        Frame_Speed_dpkg frame;

        frame.header[0] = Header1;
        frame.header[1] = Header2;
        frame.len = speed_dpkg_len;
        frame.crc_head = 0;
        
        frame.speed.data_tag = speed_tag;
        frame.speed.velocity = speed*1000;
        frame.speed.yaw =yaw*1000;
        frame.crc_dpkg= 0;

        std::vector<uint8_t> Bytes = structPack_Bytes<Frame_Speed_dpkg>(frame);

        uint8_t CRC8  = crc_8(Bytes.data(),FRAME_CALCU_CRC_BYTES);
        uint16_t CRC16 = crc_16(Bytes.data()+FRAME_INFO_SIZE,speed_dpkg_len);

        /* reset crc value  */
        setBuffCRCValue(Bytes,FRAME_DPKG_LEN_OFFSET,CRC8);
        setBuffCRCValue<uint16_t,2>(Bytes,FRAME_INFO_SIZE+speed_dpkg_len,CRC16);
        // ROS_INFO("%d", CRC16);
        // ROS_INFO("%d", Bytes2Num<uint16_t,2>(Bytes.data()+FRAME_INFO_SIZE+speed_dpkg_len));

        // for(int i=0;i<Bytes.size();++i)
        //     ROS_INFO("%d", Bytes[i]);
        // ROS_INFO("%x", Bytes[5]);      //data_tag
        // ROS_INFO("%.1f",(float) Bytes2Num<int16_t,2>(&Bytes[6])/1000.0);
        // ROS_INFO("%.1f",(float) Bytes2Num<int16_t,2>(&Bytes[8])/1000.0);
        // ROS_INFO("%d",Bytes2Num<uint16_t,2>(&Bytes[10]));    //crc16

        size_t send_count=sp_.write(Bytes);
        return send_count==Bytes.size();
    }
}