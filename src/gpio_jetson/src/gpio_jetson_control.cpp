//
// Created by eldar on 01.03.2020.
//

#include <std_msgs/String.h>
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpio_control");
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<std_msgs::String>("gpio", 1000);
    while (ros::ok())
    {
        std::string ss;
        std_msgs::String msg;
        char c;
        std::cin >> c;
        switch (c) {
            case 'w':
                ROS_INFO("forward");
                ss = "forward";
                break;
            case 'a':
                ROS_INFO("left");
                ss = "left";
                break;
            case 's':
                ROS_INFO("backward");
                ss = "backward";
                break;
            case 'd':
                ROS_INFO("right");
                ss = "right";
                break;
            default:
                ROS_INFO("stop");
                ss = "stop";
                break;
        }
        msg.data = ss;
        publisher.publish(msg);
        ros::spinOnce();
    }
    return 0;
}