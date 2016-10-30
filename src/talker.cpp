#include <string>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include <std_msgs/Int32.h>              //ros定义的Int32数据类型
#include <std_msgs/String.h>
#include <sstream>
#include <unistd.h>
#include <serial/serial.h>

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

unsigned char buf[2048]={0};                      //定义字符串长度
unsigned char buf1[2048];                      //定义字符串长度

int main(int argc, char** argv) {

    ros::init(argc, argv, "talker");       //初始化节点
    ros::NodeHandle n;
    
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("lingxilingxi", 1000);      //定义发布消息的名称及sulv

    ros::Rate loop_rate(1);


    //    io_service iosev;
    //    serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
    //    sp.set_option(serial_port::baud_rate(115200));
    //    sp.set_option(serial_port::flow_control());
    //    sp.set_option(serial_port::parity());
    //    sp.set_option(serial_port::stop_bits());
    //    sp.set_option(serial_port::character_size(8));


    serial::Serial sp("/dev/ttyUSB0",115200, serial::Timeout::simpleTimeout(1000));

    while (ros::ok())
    {
        // write(sp, buffer(buf1, 6));  //write the speed for cmd_val
        //read (sp,buffer(buf));
        int size = 0;
        size = sp.available();
        if(size==0)
        {
            continue;

        }
        sp.read(buf,size);
        string str(&buf[0],&buf[size]);            //将数组转化为字符串
        //string str(buf);            //将数组转化为字符串
        //if(buf[0]=='p' && buf[21] == 'a')
        // {
        std_msgs::String msg;
        std_msgs::Int32 msg_Int32;
        std::stringstream ss;
        ss <<str;

        msg.data = ss.str();
        int npos=msg.data.find_first_of("WAKE UP!angle:");
        //int npos=msg.data.find_first_of("WAKE UP!angle:");
        int count=0;
        if(npos<0)
        {
            continue;
        }
        for(int i = npos+14;i<npos+18;i++)
        {

            if(msg.data[i]<='9'&&msg.data[i]>='0')
            {
                count++;
                cout<<msg.data[i]<<endl;
            }
            else
            {break;}

        }
        std::string str_angle = msg.data.substr(npos+14, npos+14+count);
        atoi(str_angle.c_str());
        //ROS_INFO("str_angle=%s", str_angle.c_str());
        //if(angle!=0)
        //{
        //}

        count=0;
        int npos1=msg.data.find_first_of("\n");
        //std::string str_angle = msg.data.substr(npos+6, msg.data.size()-npos-15);
        int angle = atoi(str_angle.c_str());
        if(angle>360)
        {
            continue;
        }
        msg_Int32.data = angle;
        ROS_INFO("angle=%d", angle);
        /*ROS_INFO("npos=%d", npos);
ROS_INFO("npos1=%d", npos1);
ROS_INFO("str_angle=%s", str_angle.c_str());*/


        ROS_INFO("%s", msg.data.c_str());//打印接受到的字符串
        if(angle!=0)
        {
            msg.data=str_angle;
            chatter_pub.publish(msg_Int32);   //发布消息
        }


        //write(sp, buffer("RESET\r", 6));
        //sp.write("RESET\r",6);
        //int size1 = sp.available();

        //read (sp,buffer(buf1));
        ros::spinOnce();

        loop_rate.sleep();


        //sleep(10000);
        //break;
    }
    //iosev.run();
    
    return 0;

}
