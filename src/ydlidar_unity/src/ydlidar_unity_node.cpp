//
// Created by eldar on 19.05.2020.
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

std::vector<float> unityPointCloud;
sensor_msgs::LaserScan lastX4Message;

void x4Callback(const sensor_msgs::LaserScan x4Cloud)
{
    for (int i = 540; i < 720; i++)
    {
        unityPointCloud[i] = x4Cloud.ranges[i];
    }
    for (int i = 0; i < 180; i++)
    {
        unityPointCloud[i] = x4Cloud.ranges[i];
    }
    lastX4Message = x4Cloud;
    ROS_INFO("X4");
}

void f4Callback(const sensor_msgs::LaserScan::ConstPtr f4Cloud)
{
    int j = 180;
    for (int i = 540; i < 720; i++)
    {
        unityPointCloud[j] = f4Cloud->ranges[i];
        j++;
    }
    for (int i = 0; i < 180; i++)
    {
        unityPointCloud[j] = f4Cloud->ranges[i];
        j++;
    }
    ROS_INFO("F4 j = %d", j);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ydlidar_unity_node");
    ros::NodeHandle nodeHandle;
    const ros::Subscriber f4Subscriber = nodeHandle.subscribe("scanF4", 1000, f4Callback);
    const ros::Subscriber x4Subscriber = nodeHandle.subscribe("scanX4", 1000, x4Callback);
    const ros::Publisher unityPublisher = nodeHandle.advertise<sensor_msgs::LaserScan>("scan", 1000);
    while (ros::ok())
    {
        /*lastX4Message.ranges = unityPointCloud;
        unityPublisher.publish(lastX4Message);
        for (int i = 0; i < 180; ++i) {
            std::cout << lastX4Message.ranges[i] << " ";
        }
        std::cout << std::endl;*/
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}