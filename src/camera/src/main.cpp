/**
 * Run example:
 *  
 * rosrun camera camera _index:=0 
 * 
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"
#include <iostream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mike_camera");
    ros::NodeHandle nh("~");

    int index;
    nh.getParam("index", index);
    ROS_INFO("Index = %d", index);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("raw", 2, false);
    cv::VideoCapture cap(index);

    if(!cap.isOpened()){ // Check if camera opened successfully
        ROS_ERROR("Error opening video stream or file");
        return 1;
    }

    while(ros::ok()){
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            ROS_ERROR("Get empty frame");
        }

        cv_bridge::CvImage lo_img;

        lo_img.encoding = "bgr8";
        lo_img.image = frame;

        pub.publish(lo_img.toImageMsg());
	
        ROS_INFO("Frame published");
	
        ros::spinOnce();
    }
}

