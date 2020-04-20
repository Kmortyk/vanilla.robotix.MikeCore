//
// Created by eldar on 20.04.2020.
//

#include "ros/ros.h"
#include "../../gpio_jetson_service/include/gpio_jetson_service/commands.hpp"
#include "pcl_ros/point_cloud.h"
#include "gpio_jetson_service/gpio_srv.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>

bool backward, left, forward, right;
float backward_m = 0, left_m = 0, forward_m = 0, right_m = 0;
ros::ServiceClient gpio_client;

void ydLidarPointsCallback(const sensor_msgs::LaserScanConstPtr& message) {
    float backward_lm = 0, left_lm = 0, forward_lm = 0, right_lm = 0;
    for (int i = 0; i < 720; ++i) {
        if (message->ranges[i] > 1) {
            ROS_WARN("Range on %d > 1 !!! and equals %f", i, message->ranges[i]);
        }
        if (i > 270 && i < 450) {
            backward_lm += message->ranges[i] > 0 ? message->ranges[i] : 1;
        } else
        if (i > 90 && i < 270) {
            left_lm += message->ranges[i] > 0 ? message->ranges[i] : 1;
        } else
        if (i > 630 || i < 90) {
            forward_lm += message->ranges[i] > 0 ? message->ranges[i] : 1;
        } else
        if (i > 450 && i < 630) {
            right_lm += message->ranges[i] > 0 ? message->ranges[i] : 1;
        }
    }
    backward_m = backward_lm / 180;
    left_m = left_lm / 180;
    forward_m = forward_lm / 180;
    right_m = right_lm / 180;
    /*if (message->ranges[360] > 0 && message->ranges[360] < 0.2f) {
    }*/
    for (int i = 0; i < 720; i++) {
        left = right = backward = forward = false;
        if (message->ranges[i] > 0 && message->ranges[i] < 0.3f) {
            if (i > 270 && i < 450) {
                ROS_WARN("Backward!");
                backward = true;
                return;
            } else
            if (i > 90 && i < 270) {
                ROS_WARN("Left!");
                left = true;
                return;
            } else
            if (i > 630 || i < 90) {
                ROS_WARN("Forward!");
                forward = true;
                return;
            } else
            if (i > 450 && i < 630) {
                ROS_WARN("Right!");
                right = true;
                return;
            }
        }
    }
}

void gpio_command(const uint8_t command) {
    gpio_jetson_service::gpio_srv service;
    service.request.command = command;
    gpio_client.call(service);
}

void movement() {
    if (forward) {
        gpio_command(MoveCommands::FULL_STOP);
    } else {
        gpio_command(MoveCommands::FORWARD_LOW);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "movement");
    ros::NodeHandle nodeHandle;
    ros::Subscriber ydlidarPointsSub =
            nodeHandle.subscribe<sensor_msgs::LaserScan>("/scan", 1000, ydLidarPointsCallback);
    gpio_client = nodeHandle.serviceClient<gpio_jetson_service::gpio_srv>("gpio_jetson_service");
    gpio_jetson_service::gpio_srv service;
    service.request.command = MoveCommands::FULL_STOP;
    gpio_client.call(service);
    while (ros::ok()) {
        //movement();
        ROS_INFO("Forward: %f, Left: %f, Right: %f, Backward: %f", forward_m, left_m, right_m, backward_m);
        ros::spinOnce();
    }
    gpio_command(MoveCommands::FULL_STOP);
    return EXIT_SUCCESS;
}