#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <cstdlib>
#include <ctime>
#include "geometry_msgs/Twist.h"

void randomPubTwist(ros::Publisher &pub);

int main(int argc, char **argv)
{
	ros::init(argc,argv,"raspbot_test_node");
    ros::NodeHandle nh("");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Rate r(20);

    while(ros::ok)
    {
        randomPubTwist(pub);
        r.sleep();
        ros::spinOnce();
    }

}

void randomPubTwist(ros::Publisher &pub)
{
    std::srand(time(nullptr));

    float speed = (std::rand()%(10000+10000)-10000)/1000.0;
    float yaw = (std::rand()%(10000+10000)-10000)/1000.0;

    geometry_msgs::Twist twist;

    twist.linear.x=speed;
    twist.linear.y=0;
    twist.linear.z=0;

    twist.angular.x=0;
    twist.angular.y=0;
    twist.angular.z=yaw;

    pub.publish(twist);

}


// 1175. 质数排列

// 请你帮忙给从 1 到 n 的数设计排列方案，使得所有的「质数」都应该被放在「质数索引」（索引从 1 开始）上；你需要返回可能的方案总数。

// 让我们一起来回顾一下「质数」：质数一定是大于 1 的，并且不能用两个小于它的正整数的乘积来表示。

// 由于答案可能会很大，所以请你返回答案 模 mod 10^9 + 7 之后的结果即可。

 

// 示例 1：

// 输入：n = 5
// 输出：12
// 解释：举个例子，[1,2,5,4,3] 是一个有效的排列，但 [5,2,3,4,1] 不是，因为在第二种情况里质数 5 被错误地放在索引为 1 的位置上。

// 示例 2：

// 输入：n = 100
// 输出：682289015

 

// 提示：

//     1 <= n <= 100


class Solution {
public:
    int numPrimeArrangements(int n) {

    }
};

