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
#include "nav_msgs/Path.h"
#include "sensor_msgs/Imu.h"
#include "raspbot_bringup/raspbot_comm.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "raspbot_msgs/bot_speed.h"
#include "raspbot_msgs/bot_encoder_debug.h"

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
     *               0    帧缓冲未完成
     *               1    解析成功
     *               -1   帧头错误
     *               -2   校验错误
     *               -3   数据域长度出错
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
     */
    void publishIMU();

    /**
     * @brief 计算里程计
     *
     */
    void publishTransformAndOdom();

    /**
     * @brief 发布速度
     * 
     */
    void publishSpeed();

    /**
     * @brief 发布编码器
     * 
     */
    void publishEncoderDebug(double linear,double angular);

    bool sendFrame_Speed_dpkg(double speed=0.0,double yaw=0.0);

private:

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
     * @brief  car params
     */
    double raw_wheel_pose_x_;
    double raw_wheel_pose_y_;
    double raw_wheel_pose_theta_;
    double linearSpeed_;
    double angularSpeed_;
    double turnRadius_;


    /**
     * @brief 里程计发布管理
     */
    ros::Publisher      odom_pub_;
    nav_msgs::Odometry  wheel_odom_;
    std::string         odom_topic_;
    bool                publish_odomTF_;

    /**
     * @brief 轨迹路径发布管理
     */
    ros::Publisher      wheel_path_pub_;
    nav_msgs::Path      wheel_path_;
    std::string         wheel_path_topic_;
    bool                publish_wheel_path_;

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
     * @brief 速度发布管理
     */
    ros::Publisher    speed_pub_;
    std::string       speed_topic_; 
    raspbot_msgs::bot_speed   speed_;
    bool              publish_speed_;

    /**
     * @brief encoder_debug发布管理
     */
    ros::Publisher    encoder_debug_pub_;
    std::string       encoder_debug_topic_; 
    raspbot_msgs::bot_encoder_debug   encoder_debug_;
    bool              publish_encoder_debug_;

    /**
     * @brief tf坐标变换及广播发布
     */
    geometry_msgs::TransformStamped  odomtfs_;
    tf2_ros::TransformBroadcaster    tfBroadcaster_;



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
     * @brief         serial --> buff --> frame
     */
    Stream_msgs       stream_msgs;
    /**
     * @brief         参数
     */
    Robot_msgs        robot_msgs;


    /**
     * @brief serial data update flag
     */
    bool imu_updated;
    bool encoder_updated;

};



};



#endif