//
// Created by eldar on 23.02.2020.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

void ydLidarPointsCallback(const std_msgs::String::ConstPtr& message) {
    ROS_INFO("Gotcha: [%s]", message->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "movement");
    ros::NodeHandle nodeHandle;
    ros::Subscriber ydlidarPointsSub =
            nodeHandle.subscribe("/scan_matched_points2", 1000, ydLidarPointsCallback);
    ros::spin();
}