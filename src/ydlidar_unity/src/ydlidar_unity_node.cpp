//
// Created by eldar on 19.05.2020.
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

float unityPointCloud[720];
sensor_msgs::LaserScan message;

void x4Callback(const sensor_msgs::LaserScan x4Cloud)
{
    for (int i = 540; i < 720; i++)
    {
        unityPointCloud[i] = x4Cloud.ranges[i];
    }
    for (int i = 0; i < 181; i++)
    {
        unityPointCloud[i] = x4Cloud.ranges[i];
    }
    message.angle_increment = x4Cloud.angle_increment;
    message.angle_max = x4Cloud.angle_max;
    message.angle_min = x4Cloud.angle_min;
    message.header = x4Cloud.header;
    message.intensities = x4Cloud.intensities;
    message.range_max = x4Cloud.range_max;
    message.range_min = x4Cloud.range_min;
    message.scan_time = x4Cloud.scan_time;
    message.time_increment = x4Cloud.time_increment;
    //std::cout << std::endl;
}

void f4Callback(const sensor_msgs::LaserScan::ConstPtr f4Cloud)
{
    int j = 181;
    for (int i = 541; i < 720; i++)
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
        message.ranges.clear();
        for (float & i : unityPointCloud)
        {
            message.ranges.push_back(i);
        }
        for (auto i: message.ranges)
            std::cout << i << ' ';
        std::cout << std::endl << std::endl;
        unityPublisher.publish(message);
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}