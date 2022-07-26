#include <ros/ros.h>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


int main(int argc, char **argv)
{
	ros::init(argc,argv,"raspbot_test_node");
    ros::NodeHandle nh("");

    ros::Rate r(50);

  
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    
    geometry_msgs::TransformStamped  odomtfs;
    nav_msgs::Odometry odom_msg;
    tf2_ros::TransformBroadcaster    tfBroadcaster;

    double x=0,y=0,yaw = 0.0;
    // double length=0.2;
    // double width=0.2;

    while(ros::ok)
    {
        odomtfs.header.frame_id = "odom";
        odomtfs.header.stamp = ros::Time::now();

        odomtfs.child_frame_id = "base_link";
        

        odomtfs.transform.translation.x += 0.001;
        odomtfs.transform.translation.y += 0.001;
        odomtfs.transform.translation.z = 0.0;


        tf2::Quaternion qtn;
        qtn.setRPY(0,0,0);
        odomtfs.transform.rotation.w = qtn.getW();
        odomtfs.transform.rotation.x = qtn.getX();
        odomtfs.transform.rotation.y = qtn.getY();
        odomtfs.transform.rotation.z = qtn.getZ();

        tfBroadcaster.sendTransform(odomtfs);  //publsih

        odom_msg.header.frame_id = "odom";
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x += 0.001;
        odom_msg.pose.pose.position.y += 0.001 ;
        odom_msg.pose.pose.position.z = 0.0;


        // odom_msg.pose.pose.orientation.x = qtn.getX();
        // odom_msg.pose.pose.orientation.y = qtn.getY();
        // odom_msg.pose.pose.orientation.z = qtn.getZ();
        // odom_msg.pose.pose.orientation.w = qtn.getW();

        // odom_msg.twist.twist.linear.x = 0.1;

        odom_pub.publish(odom_msg);

        r.sleep();
        ros::spinOnce();
    }
    

}
