//
// Created by eldar on 19.05.2020.
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

float unityPointCloud[720];
sensor_msgs::LaserScan lastX4Message;

void x4Callback(const sensor_msgs::LaserScan x4Cloud)
{
    for (int i = 540; i < 720; i++)
    {
        unityPointCloud[i] = x4Cloud.ranges[i];
        std::cout << unityPointCloud[i] << " ";
    }
    for (int i = 0; i < 180; i++)
    {
        unityPointCloud[i] = x4Cloud.ranges[i];
        std::cout << unityPointCloud[i] << " ";
    }
    std::cout << std::endl;
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
        for (int i = 0; i < 720; i++)
        {
            lastX4Message.ranges.clear();
            lastX4Message.ranges.push_back(unityPointCloud[i]);
            //std::cout << lastX4Message.ranges[i] << " ";
        }
        unityPublisher.publish(lastX4Message);
        //std::cout << std::endl;
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}