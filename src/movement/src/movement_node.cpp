//
// Created by eldar on 23.02.2020.
//

#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"

void ydLidarPointsCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    pcl::PointCloud<pcl::PointXYZ> depth;
    pcl::fromROSMsg(*message, depth);
    int x = 0, y = 0; // set x and y
    pcl::PointXYZ p1 = depth.at(x, y);
    ROS_INFO("x = %f y = %f z = %f", p1.x, p1.y, p1.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "movement");
    ros::NodeHandle nodeHandle;
    ros::Subscriber ydlidarPointsSub =
            nodeHandle.subscribe<sensor_msgs::PointCloud2>("/scan_matched_points2", 1000, ydLidarPointsCallback);
    ros::spin();
}