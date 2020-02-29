//
// Created by eldar on 23.02.2020.
//

#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"

void ydLidarPointsCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    for (int j = 0; j < message->row_step * message->height; ++j) {
        printf("%i ", message->data[j]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "movement");
    ros::NodeHandle nodeHandle;
    ros::Subscriber ydlidarPointsSub =
            nodeHandle.subscribe<sensor_msgs::PointCloud2>("/scan_matched_points2", 1000, ydLidarPointsCallback);
    ros::spin();
}