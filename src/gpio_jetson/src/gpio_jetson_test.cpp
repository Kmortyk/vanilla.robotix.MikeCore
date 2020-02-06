//
// Created by eldar on 02.02.2020.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpio_jetson_test");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("gpio", 1000);
    ros::Rate loop_rate(1);
    int count = 0;
    std::string directions[] = {"forward", "backward", "left", "right"};
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << directions[count % 4];
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}