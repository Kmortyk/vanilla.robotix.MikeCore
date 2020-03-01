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
    std::stringstream ss;
    while (ros::ok())
    {
        std_msgs::String msg;
        char c;
        std::cin >> c;
        switch (c) {
            case 'w':
                ss << "forward";
                break;
            case 'a':
                ss << "left";
                break;
            case 's':
                ss << "backward";
                break;
            case 'd':
                ss << "right";
                break;
            default:
                ss << "stop";
                break;
        }
        msg.data = ss.str();
        publisher.publish(msg);
        ss.clear();
        ros::spinOnce();
    }
    return 0;
}