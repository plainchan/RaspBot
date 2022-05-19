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

#ifndef _RASPBOT_BASE_H_
#define _RASPBOT_BASE_H_

#include "ros/ros.h"
#include "serial/serial.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "raspbot_bringup/raspbot_comm.hpp"

// #define debug



namespace raspbot
{

class BotBase
{

public:

    /**
     * @brief Construct a new BotBase object
     * 
     * @param nh 
     * @param private_nh 
     */
    BotBase(ros::NodeHandle nh,ros::NodeHandle private_nh);

    /**
     * @brief Destroy the BotBase object
     * 
     */
    ~BotBase();

    /**
     * @brief  初始化
     * 
     */
    void initialize();

    /**
     * @brief  初始化串口配置
     * 
     * @return true 
     * @return false 
     */
    bool serial_init();

    /**
     * @brief 回调函数
     * 
     * @param msg_ptr 
     */
    void speedTwistCallBack(const geometry_msgs::Twist::ConstPtr &msg_ptr);
    
    /**
     * @brief  设置参数
     * 
     */
    void setting();

    /**
     * @brief 定时器回调函数
     * 
     * @param event 
     */
    void periodicUpdate(const ros::TimerEvent &event);

    /**
     * @brief    将数据流缓冲到Buff,数据流中可能只包含半帧或者多帧数据
     * 
     * @param[out] stream_msgs 
     * @param[in]  buff 
     * @param[in]  size 
     * @return int 
     *               1    解析成功
     *               -1   帧头错误
     *               0    帧缓冲未完成
     *               -2   数据域长度出错
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

    /**
     * @brief Set the Imu Value object
     * 
     * @param imu 
     */
    void setImuValue(sensor_msgs::Imu &imu);

    /**
     * @brief 计算里程计
     * 
     * @param odom 
     */
    void calcuOdomValue(nav_msgs::Odometry &odom);

protected:

    /**
     * @brief Nodehandle for publisher and subscriber
     *   nh_        
     *   nhPrivate_   参数服务器
     */
    ros::NodeHandle   nh_;
    ros::NodeHandle   nhPrivate_;

    /**
     * @brief  定时器
     */
    ros::Timer        periodicUpdateTimer_;
    double            frequency_;

    /**
     * @brief 里程计发布管理
     */
    ros::Publisher      odom_pub_;
    nav_msgs::Odometry  wheel_odom_;
    std::string         odom_topic_;
    bool                publish_odom_;

    /**
     * @brief IMU发布管理
     * 
     */
    ros::Publisher    imu_pub_;
    sensor_msgs::Imu  imu_;
    std::string       imu_topic_;

    /**
     * @brief Twist订阅管理
     */
    ros::Subscriber   twist_sub_;
    std::string       twist_topic_;


    /**
     * @brief         frame id
     */
    std::string       base_frame_;
    std::string       odom_frame_;
    std::string       imu_frame_;

    /**
     * @brief         串口
     */
    serial::Serial    sp_;
    std::string       udev_port_;
    int               baud_;

    

    /**
     * @brief 
     */
    Stream_msgs       stream_msgs={};
};



};



#endif