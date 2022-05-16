#ifndef _RASPBOT_BASE_H_
#define _RASPBOT_BASE_H_

#include "ros/ros.h"
#include "serial/serial.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "raspbot_bringup/raspbot_comm.hpp"
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
    std::string       imu_frame_;
    std::string       udev_port_;
    int               baud_;

    double            frequency_;

    bool              publish_odom_;
    ros::Timer        periodicUpdateTimer_;

    sensor_msgs::Imu  imu_;
    void setImuValue(sensor_msgs::Imu &imu);

    nav_msgs::Odometry wheel_odom_;
    void calcuOdomValue(nav_msgs::Odometry &odom);


    uint16_t          bytesCount_;
    Stream_msgs       stream_msgs;

    /**
     * @brief    将数据流缓冲到Buff,数据流中可能只包含半帧或者多帧数据
     * 
     * @param[out] stream_msgs 
     * @param[in]  buff 
     * @param[in]  size 
     * @return int 
     *         1   解析成功
     *         -1  帧头错误
     *         0   校验错误
     *         -2  数据域长度出错
     */
    int parse_stream(Stream_msgs &stream_msgs,const uint8_t buff);

    /**
     * @brief 对一帧数据解码
     * 
     * @param[out] stream_msgs 
     * @param[out] buff
     * @return int
     *          1   解析成功
     *          -1  帧错误
     */
    int decode_frame(Stream_msgs &stream_msgsW);

    Bytes2U16   B2U16;
    Bytes2Float B2F;
    Bytes2INT16 B2INT16;
};



};



#endif