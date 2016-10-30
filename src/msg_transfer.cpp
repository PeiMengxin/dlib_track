#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>

using namespace std;

double limit(double val, double min_v, double max_v)
{
    return (val<min_v?min_v:(val>max_v?max_v:val));
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

 void chatterCallback(const geometry_msgs::Twist msg)
{
    // ROS_INFO("point: [%d  %d], x_diff: [%d]", msg.x, msg.y, msg.x-160);
    // [msg.linear.x, msg.linear.y, msg.linear.z]-->[x, y, wave_dis]
    // [msg.angular.x, msg.angular.y, msg.angular.z]-->[speed_v_max, y, speed_ang_max]
    if(msg.linear.x<0)
    {
        return;
    }

    static ros::NodeHandle n1;
    static ros::Publisher pub = n1.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    geometry_msgs::Twist msg_Twist;
    msg_Twist.linear.x = 0;
    msg_Twist.linear.y = 0;
    msg_Twist.linear.z = 0;
    msg_Twist.angular.x = 0;
    msg_Twist.angular.y = 0;
    msg_Twist.angular.z = 0;

    double x_diff = 160-msg.linear.x;
    msg_Twist.angular.z = x_diff/160;

    if(msg_Twist.angular.z>(msg.angular.z/100))
    {
        msg_Twist.angular.z=(msg.angular.z/100);
    }
    if(msg_Twist.angular.z<-(msg.angular.z/100))
    {
        msg_Twist.angular.z=-(msg.angular.z/100);
    }

    if(msg.linear.z>1000)
    {
        msg_Twist.linear.x = (msg.linear.z-1000)/1000;
        msg_Twist.linear.x = limit(msg_Twist.linear.x, 0, msg.angular.x/100);
    }

    ROS_INFO("[speed_v, speed_ang]:[%.1f %.1f]",  msg_Twist.linear.x, msg_Twist.angular.z);

    pub.publish(msg_Twist);
    return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "msg_transfer");

  ros::NodeHandle n;

  //ros::Rate loop_rate(10);

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();
  
  return 0;
}
