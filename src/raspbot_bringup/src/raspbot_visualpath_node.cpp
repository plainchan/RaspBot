#include <ros/ros.h>
#include <iostream>
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"


void OdomCallBack(const nav_msgs::Odometry::ConstPtr &msg);

nav_msgs::Path      wheel_path;
std::string         odom_frame;
ros::Publisher      wheel_path_pub;


int main(int argc, char **argv)
{
	ros::init(argc,argv,"raspbot_visualpath_node");
    ros::NodeHandle nh(""),_nh("~");

    std::string  wheel_path_topic = _nh.param<std::string>("wheel_path_topic","wheel/path");
    std::string  odom_topic = _nh.param<std::string>("odom_topic","wheel/odom");
    odom_frame = _nh.param<std::string>("odom_frame","odom");


    wheel_path.header.frame_id = odom_frame;
    wheel_path.header.stamp = ros::Time::now();


    nh.subscribe<nav_msgs::Odometry>(odom_topic, 50, &OdomCallBack);
    wheel_path_pub= nh.advertise<nav_msgs::Path>(wheel_path_topic,1000);


    while(ros::ok)
    {
        wheel_path_pub.publish(wheel_path);
        ros::spinOnce();
    }
    

}

/**
 * @brief publish wheel path
 * @attention Path msg is dynamic vector
 *            and occpuy memory more and more as time goes by
 * 
 */

void OdomCallBack(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    if(odom_msg->pose.pose == wheel_path.poses.back().pose)
        return;
    geometry_msgs::PoseStamped cur_pose;
    cur_pose.header.frame_id = odom_frame;
    cur_pose.header.stamp = ros::Time::now();

    cur_pose.pose.position= odom_msg->pose.pose.position;
    cur_pose.pose.orientation= odom_msg->pose.pose.orientation;
    wheel_path.poses.emplace_back(cur_pose);
}
