#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <iostream>

void callback(const sensor_msgs::ImageConstPtr& msg)
{
   ROS_INFO("Get image");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mike_hard_drive");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("jetbot_camera/raw", 1000, callback);
    ros::spin();
}
