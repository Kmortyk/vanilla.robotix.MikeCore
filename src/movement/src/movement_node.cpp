//
// Created by eldar on 23.02.2020.
//

#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "gpio_jetson_service/gpio_srv.h"
//FIX IT
#include "../../gpio_jetson_service/include/gpio_jetson_service/commands.hpp"
#include <vector>

bool crashing = false;

void ydLidarPointsCallback(const sensor_msgs::LaserScanConstPtr& message) {
    if (message->ranges[360] > 0 && message->ranges[360] < 0.2f) {
        ROS_WARN("DANGER!!");
        crashing = true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "movement");
    ros::NodeHandle nodeHandle;
    ros::Subscriber ydlidarPointsSub =
            nodeHandle.subscribe<sensor_msgs::LaserScan>("/scan", 1000, ydLidarPointsCallback);
    ros::ServiceClient client = nodeHandle.serviceClient<gpio_jetson_service::gpio_srv>("gpio_jetson_service");
    gpio_jetson_service::gpio_srv service;
    service.request.command = MoveCommands::FORWARD_FAST;
    client.call(service);
    while (ros::ok()) {
        if (crashing) {
            service.request.command = MoveCommands::FULL_STOP;
            client.call(service);
            sleep(1);
            service.request.command = MoveCommands::RIGHT_FORWARD_FAST;
            client.call(service);
            sleep(1);
            service.request.command = MoveCommands::RIGHT_STOP;
            client.call(service);
            sleep(1);
            crashing = false;
            service.request.command = MoveCommands::FORWARD_FAST;
            client.call(service);
        } else {
        }
        ros::spinOnce();
    }
    service.request.command = MoveCommands::FULL_STOP;
    client.call(service);
    return EXIT_SUCCESS;
}