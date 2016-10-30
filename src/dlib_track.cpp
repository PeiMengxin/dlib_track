#include <dlib/geometry.h>
#include <dlib/matrix.h>
#include <dlib/array2d.h>
#include <dlib/image_transforms/assign_image.h>
#include <dlib/opencv.h>
#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <iostream>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <thread>

using namespace dlib;
using namespace std;
using namespace cv;

bool select_flag = false;
bool lost_target_flag = false;
Point pt_origin;
Point pt_end;

int linear_max = 100;
int linear_speed = 20;
int angular_max = 20;
int lost_number = 0;
// serial::Serial serial_port("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(1000));
serial::Serial serial_port;

void sleep_ms(int ms)
{
    usleep(1000*ms);
    lost_number++;
    ROS_INFO("lost:%d", lost_number);
}

void lostWarning()
{
    ROS_INFO("Warning: Lost Target!");
    cout<<"\a"<<endl;
}

dlib::rectangle getInitPosition(cv::Rect r)
{
    //dlib::rectangle initPos(r.x, r.y, r.x + r.width, r.y + r.height);
    return dlib::rectangle(r.x, r.y, r.x + r.width, r.y + r.height);
}

cv::Rect dlibRect2CVRect(dlib::rectangle r)
{
    return cv::Rect(r.left(), r.top(), r.width(), r.height());
}

unsigned char serial_buffer[8]={0};
int wave_dis = 0;

char start_order = 0x55;
bool isTerminal = false;

void ultraWaveDistance()
{
    int dis_temp = 0;
    double factor = 0.5;

    if(!serial_port.isOpen())
    {
        cout<<"serial port read failed!"<<endl;
        return;
    }

    while(!isTerminal)
    {
        serial_port.write(&start_order);
        int size = serial_port.available();
        if(size==2)
        {
            serial_port.read(serial_buffer,size);

            dis_temp = serial_buffer[0]*256+serial_buffer[1];
            wave_dis = wave_dis*factor+dis_temp*(1-factor);

        }

        usleep(1000*20);
    }

}


void help()
{
    cout << "--Choose the video source and put Enter: " << endl;
    cout << "	0: choose video frome file" << endl;
    cout << "	1: choose video frome camera" << endl;
}

void onMouse(int event, int x, int y, int, void* param)
{
    if (select_flag)
    {
        ((Rect*)(param))->x = MIN(pt_origin.x, x);
        ((Rect*)(param))->y = MIN(pt_origin.y, y);
        ((Rect*)(param))->width = abs(x - pt_origin.x);
        ((Rect*)(param))->height = abs(y - pt_origin.y);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        select_flag = true;
        pt_origin = Point(x, y);
        *(Rect*)(param) = Rect(x, y, 0, 0);
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        pt_end = Point(x, y);
        select_flag = false;
    }
}

cv::Point getCenter(cv::Rect r)
{
    cv::Point p(0,0);
    p.x = (r.tl().x+r.br().x)/2;
    p.y = (r.tl().y+r.br().y)/2;
    return p;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("chatter", 1000);

    ros::Rate loop_rate(10);

    serial_port.setPort("/dev/ttyUSB0");
    serial_port.setBaudrate(9600);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial_port.setTimeout(timeout);

    serial_port.open();
    usleep(1000*1000);

    while(!serial_port.isOpen())
    {
        static int n = 0;
        cout<<"serial_port open failed!"<<endl;
        usleep(1000*100);
        n++;
        if(n>10)
            break;
    }

    std::thread ultra_wave_distance_thread(ultraWaveDistance);

    cv::namedWindow("bar");
    cv::createTrackbar("linear_max","bar",&linear_max,200);
    cv::createTrackbar("angular_max","bar",&angular_max,40);
   
    VideoCapture cap;
    bool start_track = false;

    //cap.open("ir.avi");
    cap.open(0);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
    waitKey(1000);
    if (!cap.isOpened())
    {
        cout << endl;
        cout << "*********************************************" << endl;
        cout<< "capture open failed!"<<endl;
        cout << "*********************************************" << endl;
        cout << endl;
        return -1;
    }

    Mat imgSrc;//source image
    Mat imgSrcCopy;//source copy

    // Load the first frame.
    cap >> imgSrc;
    if (NULL == imgSrc.data)
    {
        cout << endl;
        cout << "*********************************************" << endl;
        cout << "capture open failed!"<<endl;
        cout << "*********************************************" << endl;
        cout << endl;
        return -1;
    }

    imgSrc.copyTo(imgSrcCopy);

    imshow("tracker", imgSrcCopy);

    Rect *pselect = new Rect;
    setMouseCallback("tracker", onMouse, pselect);//mouse interface

    cout << "\n--Choose a object to track use mouse" << endl;
    cout << "--Put 'space' to start track or put 'ESC' to exit" << endl;

    //select object roi
    while (true)
    {
        cap >> imgSrc;

        imgSrc.copyTo(imgSrcCopy);
        cv::rectangle(imgSrcCopy, *pselect, CV_RGB(0, 255, 0), 2, 8, 0);
        imshow("tracker", imgSrcCopy);

        /*if (select_flag)
        {
            waitKey(16);
            continue;
        }*/

        char c = waitKey(16);
        if (c == 32)
        {
            start_track = true;
            break;
        }
        else if (c == 27)
        {
            return 1;
        }
        else if (c == 'k')
        {
            cap >> imgSrc;;
        }
    }

    size_t frame = 1;

    // Gray img of imgSrc
    Mat imgGray;

    // Transform the source bgr image to gray
    cvtColor(imgSrc, imgGray, CV_BGR2GRAY);

    // Opencv Mat to dlib
    cv_image< unsigned char> img(imgGray);

    // Create a tracker and start a track on the juice box.
    dlib::correlation_tracker tracker;
    tracker.start_track(img, getInitPosition(*pselect));

    Mat imgShow;
    float psr = 0.0;
    // Run the tracker.  call tracker.update()
    while (true)
    {
        geometry_msgs::Twist msg_twist;
        msg_twist.linear.x = -1;
        msg_twist.linear.y = -1;
        msg_twist.linear.z = wave_dis;
        msg_twist.angular.x = linear_max;
        msg_twist.angular.z = angular_max;

        cap >> imgSrc;

        imgSrc.copyTo(imgSrcCopy);

        if (select_flag)
        {
            start_track = false;
            cv::rectangle(imgSrcCopy, *pselect, CV_RGB(0, 255, 0), 2, 8, 0);
            //imshow("tracker", imgSrcCopy);
        }
        else
        {
            if (start_track)
            {
                cvtColor(imgSrcCopy, imgGray, CV_BGR2GRAY);

                psr = tracker.update(img)*2;
                ROS_INFO("PSR:%.1f", psr);
				static int cnt_loss_track = 0, cnt_loss_track5 = 0;
				if (psr < 15)
					cnt_loss_track++;
				if (psr < 10)
					cnt_loss_track5++;
				if (cnt_loss_track > 20 || cnt_loss_track5 > 2)
				{
					cnt_loss_track = 0;
					cnt_loss_track5 = 0;
					start_track = false;
                    lost_target_flag = true;

                    continue;
				}

                cv::Rect r = dlibRect2CVRect(tracker.get_position());
                cv::rectangle(imgSrcCopy, r, CV_RGB(255, 0, 0), 2, 8, 0);
                msg_twist.linear.x = getCenter(r).x;
                msg_twist.linear.y = getCenter(r).y;
                msg_twist.linear.z = wave_dis;
            }
            else
            {
                cv::rectangle(imgSrcCopy, *pselect, CV_RGB(0, 255, 0), 2, 8, 0);
                char c = waitKey(10);
                if (c == 32)
                {
                    start_track = true;
                    lost_target_flag = false;
                    lost_number = 0;
                    tracker.start_track(img, getInitPosition(*pselect));
                }
            }

        }

        if(lost_target_flag)
        {
            lostWarning();
            sleep_ms(10);
        }

        ROS_INFO("%.1f %.1f %.1f",  msg_twist.linear.x,  msg_twist.linear.y,  msg_twist.linear.z);
        
        chatter_pub.publish(msg_twist);

        imshow("tracker", imgSrcCopy);
        char c = waitKey(10);
        if (c == 27)
        {
            break;
        }

    }

    isTerminal = true;

    ultra_wave_distance_thread.join();


    return 0;
}

