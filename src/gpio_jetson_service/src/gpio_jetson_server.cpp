//
// Created by eldar on 05.03.2020.
//

#include "ros/ros.h"
#include "gpio_jetson_server/gpio_srv.h"
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpio_jetson_server");
    return 0;
}