#include "ros/ros.h"  
#include "std_msgs/String.h"  
#include <std_msgs/Int32.h>
  
/**  
 * This tutorial demonstrates simple receipt of messages over the ROS system.  
 */  
void chatterCallback(const std_msgs::Int32 msg)
{  
  ROS_INFO("I heard: [%d]", msg.data);

 // static ros::NodeHandle n1;
 //     static ros::Publisher pub = n1.advertise<std_msgs::Int32>("lingxilingxi", 1000);

//      geometry_msgs::Twist msg_Twist;
//      msg_Twist.linear.x = 0;
//      msg_Twist.linear.y = 0;
//      msg_Twist.linear.z = 0;
//      msg_Twist.angular.x = 0;
//      msg_Twist.angular.y = 0;
//      msg_Twist.angular.z = 0;

//      msg_Twist.angular.z = x_diff/160;
//      if(msg_Twist.angular.z>(msg.angular.z/100))
//      {
//          msg_Twist.angular.z=(msg.angular.z/100);
//      }
//      if(msg_Twist.angular.z<-(msg.angular.z/100))
//      {
//          msg_Twist.angular.z=-(msg.angular.z/100);
//      }

//      if(msg.linear.z>1000)
//      {
//          msg_Twist.linear.x = (msg.angular.x/100);
//      }

      //ROS_INFO("%.1f %.1f",  msg_Twist.linear.x, msg_Twist.angular.z);

    //  pub.publish(msg);
      return;
}  
  
int main(int argc, char **argv)  
{  
  /**  
   * The ros::init() function needs to see argc and argv so that it can perform  
   * any ROS arguments and name remapping that were provided at the command line. For programmatic  
   * remappings you can use a different version of init() which takes remappings  
   * directly, but for most command-line programs, passing argc and argv is the easiest  
   * way to do it.  The third argument to init() is the name of the node.  
   *  
   * You must call one of the versions of ros::init() before using any other  
   * part of the ROS system.  
   */  
  ros::init(argc, argv, "listener");  
  
  /**  
   * NodeHandle is the main access point to communications with the ROS system.  
   * The first NodeHandle constructed will fully initialize this node, and the last  
   * NodeHandle destructed will close down the node.  
   */  
  ros::NodeHandle n;  
  
  /**  
   * The subscribe() call is how you tell ROS that you want to receive messages  
   * on a given topic.  This invokes a call to the ROS  
   * master node, which keeps a registry of who is publishing and who  
   * is subscribing.  Messages are passed to a callback function, here  
   * called chatterCallback.  subscribe() returns a Subscriber object that you  
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber  
   * object go out of scope, this callback will automatically be unsubscribed from  
   * this topic.  
   *  
   * The second parameter to the subscribe() function is the size of the message  
   * queue.  If messages are arriving faster than they are being processed, this  
   * is the number of messages that will be buffered up before beginning to throw  
   * away the oldest ones.  
   */  
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);  
  
  /**  
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all  
   * callbacks will be called from within this thread (the main one).  ros::spin()  
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.  
   */  
  ros::spin();  
  
  return 0;  
} 
