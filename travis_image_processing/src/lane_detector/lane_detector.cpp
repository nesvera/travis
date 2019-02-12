#include "ros/ros.h"
#include "std_msgs/String.h"

#include <opencv2/opencv.hpp>
#include <iostream>

#include <sstream>

using namespace cv;
using namespace std;

int main(int argc, char **argv){

    ros::init(argc, argv, "lane_detector");

    Mat img = imread("/home/nesvera/image_test.jpg", CV_LOAD_IMAGE_COLOR);

    while(1){
        //namedWindow("image", WINDOW_AUTOSIZE);
        imshow("image", img);

        waitKey(1);
    }
    return 0;

/*
    VideoCapture cap(0);

    if( !cap.isOpened() ){
        return -1;
    }

    Mat edges;
    namedWindow("edges", 1);
    for(;;){
        Mat frame;
        cap >> frame;
        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        if(waitKey(30) >= 0) break;
    }

    return 0;

    
    ros::NodeHandle n;

    ros::Publisher chartter_pub = n.advertise<std_msgs::String>("chartter", 1000);

    ros::Rate loop_rate(10);

    int count = 0;

    while(ros::ok()){

        std_msgs::String msg;

        std::stringstream ss;

        ss << "hello world" << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chartter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;

    }
    */

}