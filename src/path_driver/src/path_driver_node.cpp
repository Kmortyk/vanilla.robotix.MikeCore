//
// Created by eldar on 11/13/20.
//

#include <ros/ros.h>
#include "gpio_jetson_service/gpio_srv.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include "../../gpio_jetson_service/include/gpio_jetson_service/commands.hpp"
#include <geometry_msgs/Point.h>

#define MEASUREMENT_ROTATION 5
#define MEASUREMENT_LOCATION 10

tf::TransformListener* transformListener;
tf::StampedTransform transform_bot;
double transform_time_sec;
ros::ServiceClient gpio_client;
uint8_t currentCommand;
nav_msgs::PathConstPtr robotPath;
bool robotPathUpdated = false;

void gpio_command(const uint8_t command) {
    if (currentCommand == command)
        return;
    gpio_jetson_service::gpio_srv service;
    service.request.command = MoveCommands::FULL_STOP;
    gpio_client.call(service);
    gpio_jetson_service::gpio_srv service2;
    service2.request.command = command;
    gpio_client.call(service2);
    currentCommand = command;
}

void pathCallback(const nav_msgs::PathConstPtr& path) {
    if (path == nullptr) {
        return;
    }
    robotPath = path;
    robotPathUpdated = true;
}

double getRotationInDegrees(tf::StampedTransform stampedTransform) {
    double roll, pitch, yaw;
    stampedTransform.getBasis().getRPY(roll, pitch, yaw);
    return yaw * 180.0 / M_PI;
}

geometry_msgs::Point getRobotPose() {
    geometry_msgs::Point resultPoint;
    ros::Time now = ros::Time::now();
    if (now.toSec() - transform_time_sec < 1) {
        return resultPoint;
    }
    try {
        transformListener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
        transformListener->lookupTransform("base_link", "map", ros::Time(0), transform_bot);
    } catch (tf2::LookupException &exception) {
        ROS_ERROR("error: %s", exception.what());
        return resultPoint;
    }

    resultPoint.x = transform_bot.getOrigin().x();
    resultPoint.y = transform_bot.getOrigin().y();
    resultPoint.z = getRotationInDegrees(transform_bot);

    return resultPoint;
}

double poseToDegrees(geometry_msgs::Pose &pose) {
    tf::Quaternion quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
    tf::Matrix3x3 matrix3X3(quaternion);
    double roll, pitch, yaw;
    matrix3X3.getRPY(roll, pitch, yaw);
    return yaw * 180.0 / M_PI;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_driver");
    ros::NodeHandle nodeHandle;
    ros::Subscriber pathSubscriber =
            nodeHandle.subscribe<nav_msgs::Path>("/move_base_node/NavfnROS/plan", 1, pathCallback);
    transform_time_sec = ros::Time::now().toSec();
    transformListener = new tf::TransformListener(nodeHandle);
    gpio_client = nodeHandle.serviceClient<gpio_jetson_service::gpio_srv>("gpio_jetson_service");
    int pathIterator = 0;
    do {
        ros::spinOnce();
        if (robotPathUpdated) {
            pathIterator = 0;
            robotPathUpdated = false;
            ROS_INFO("Robot path updated");
        }
        if (robotPath == nullptr) {
//            ROS_INFO("Null");
            continue;
        }
        if (robotPath->poses.empty()) {
            ROS_INFO("Empty path!");
            gpio_command(MoveCommands::FULL_STOP);
            continue;
        }
        if (pathIterator >= robotPath->poses.size()) {
            ROS_INFO("Path end! %lu", robotPath->poses.size());
            gpio_command(MoveCommands::FULL_STOP);
            continue;
        }
        geometry_msgs::Point robotPose = getRobotPose();
        geometry_msgs::Pose targetPose = robotPath->poses[pathIterator].pose;
        double targetRotation = poseToDegrees(targetPose);
        targetRotation += 90;
        if(robotPose.z - targetRotation < MEASUREMENT_ROTATION || robotPose.z + targetRotation < MEASUREMENT_ROTATION) {
            if (robotPose.x - targetPose.position.x < MEASUREMENT_LOCATION
            || robotPose.x + targetPose.position.x < MEASUREMENT_LOCATION
            || robotPose.y - targetPose.position.y < MEASUREMENT_LOCATION
            || robotPose.y + targetPose.position.y < MEASUREMENT_LOCATION) {
                gpio_command(MoveCommands::FORWARD_LOW);
            } else {
                pathIterator++;
            }
        } else {
            // move left of right
            //ROS_INFO("Move left or right");
//            gpio_command(MoveCommands::LEFT_FORWARD_MIDDLE);
            if (targetRotation > robotPose.z) {
                gpio_command(MoveCommands::LEFT_FORWARD_LOW);
            } else {
                gpio_command(MoveCommands::RIGHT_FORWARD_LOW);
            }
        }
    } while (ros::ok());
    gpio_command(MoveCommands::FULL_STOP);
    return EXIT_SUCCESS;
}