//
// Created by eldar on 23.07.2020.
//
#include <AStar.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

const int minimalRadius = 3;

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

void findTheTargetPointAndWay() {
    for (int radius = minimalRadius; radius < 50; ++radius) {
        int iMin = -1, jMin, minCost;
        std::stack<AStar::Point2DInt> minCostWayToPoint;
        int robotX = robotPosition.position.x;
        int robotY = robotPosition.position.y;
        // Left side
        int i = robotY - radius;
        int j = robotX - radius;
        for (; i < robotY + radius; ++i) {
            if (aStar->map[i][j] == 1) {
                minCostWayToPoint = aStar->search(robotY, robotX, i, j);
                if (minCost > minCostWayToPoint.size()) {
                    minCost = minCostWayToPoint.size();
                    iMin = i;
                    jMin = j;
                }
            }
        }
        // Right side
        i = robotY - radius;
        j = robotX + radius;
        for (; i < robotY + radius; ++i) {
            if (aStar->map[i][j] == 1) {
                minCostWayToPoint = aStar->search(robotY, robotX, i, j);
                if (minCost > minCostWayToPoint.size()) {
                    minCost = minCostWayToPoint.size();
                    iMin = i;
                    jMin = j;
                }
            }
        }
        // Top side
        i = robotY - radius;
        j = robotX - radius;
        for (; j < robotX + radius; ++j) {
            if (aStar->map[i][j] == 1) {
                minCostWayToPoint = aStar->search(robotY, robotX, i, j);
                if (minCost > minCostWayToPoint.size()) {
                    minCost = minCostWayToPoint.size();
                    iMin = i;
                    jMin = j;
                }
            }
        }
        // Bottom side
        i = robotY + radius;
        j = robotX - radius;
        for (; j < robotX + radius; ++j) {
            if (aStar->map[i][j] == 1) {
                minCostWayToPoint = aStar->search(robotY, robotX, i, j);
                if (minCost > minCostWayToPoint.size()) {
                    minCost = minCostWayToPoint.size();
                    iMin = i;
                    jMin = j;
                }
            }
        }

        if (iMin > -1) {
            targetPoint.x = jMin;
            targetPoint.y = iMin;
            wayToTargetPoint = minCostWayToPoint;
            return;
        }
    }
}

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
        findTheTargetPointAndWay();
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