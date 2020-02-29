//
// Created by eldar on 23.02.2020.
//

#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"

void ydLidarPointsCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    /*pcl::PointCloud<pcl::PointXYZ> depth;
    pcl::fromROSMsg(*message, depth);
    int x = 0, y = 0; // set x and y
    pcl::PointXYZ p1 = depth.at(x, y);
    ROS_INFO("x = %f y = %f", p1.x, p1.y);*/
    //ROS_INFO("size: %lu", message->width);
    /*for (int j = 1; j < message->height; ++j) {
        for (int k = 0; k < message->row_step; ++k) {
            printf("%i ", message->data[j + k]);
        }
        //ROS_INFO("");
        printf("\n");
    }*/
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