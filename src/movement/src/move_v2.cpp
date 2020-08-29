//
// Created by eldar on 23.07.2020.
//
#include <AStar.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

struct RobotPosition {
    AStar::Point2DInt position;
    float rotation;
};

float lastMapResolution, lastWidth, lastHeight;
std::map<int, std::map<int, int8_t>> map;
AStar::Point2DInt targetPoint;
std::stack<AStar::Point2DInt> wayToTargetPoint;
RobotPosition robotPosition;
tf::TransformListener* transformListener;
AStar* aStar;

void calculateRobotPositionOnMap() {
    tf::StampedTransform transformBot;
    try {
        transformListener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
        transformListener->lookupTransform("base_link", "map", ros::Time(0), transformBot);
    } catch (tf2::LookupException &exception) {
        ROS_ERROR("error: %s", exception.what());
        return;
    }

    double tfRobotX = transformBot.getOrigin().x();
    double tfRobotY = transformBot.getOrigin().y();
    double roll, pitch, yaw;
    transformBot.getBasis().getRPY(roll, pitch, yaw);

    robotPosition.position.x = (int) (tfRobotX / lastMapResolution);
    robotPosition.position.y = (int) (tfRobotY / lastMapResolution);
    robotPosition.rotation = (float) (yaw * 180.0 / M_PI);
}

void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
    if (grid->info.resolution != lastMapResolution || grid->info.width != lastWidth || grid->info.height != lastHeight) {
        lastMapResolution = grid->info.resolution;
        lastWidth = grid->info.width;
        lastHeight = grid->info.height;
        int i = 0;
        for (int8_t number : grid->data) {
            for (int j = 0; j < grid->info.width; ++j) {
                map[i][j] = number;
                if (number > 80) {
                    aStar->map[i][j] = 0;
                } else {
                    aStar->map[i][j] = 1;
                }
            }
            i++;
        }
        aStar->iMax = grid->info.height;
        aStar->jMax = grid->info.width;
        if (i != grid->info.height - 1) {
            ROS_WARN("Map array height != map height");
        }
    }
    calculateRobotPositionOnMap();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_v2");
    ros::NodeHandle nodeHandle;

    aStar = new AStar();

    transformListener = new tf::TransformListener(nodeHandle);

    ros::Subscriber mapSubscriber = nodeHandle.subscribe("map", 1000, occupancyGridCallback);

    while (ros::ok()) {
        ros::spinOnce();
        ROS_INFO("Target x: %d; Target y: %d", targetPoint.x, targetPoint.y);
        ROS_INFO("Last map resolution: %f", lastMapResolution);
        ROS_INFO("Robot position x = %d y = %d", robotPosition.position.x, robotPosition.position.y);
        ROS_INFO("Size of the way: %lu", wayToTargetPoint.size());
    }
    return EXIT_SUCCESS;
}