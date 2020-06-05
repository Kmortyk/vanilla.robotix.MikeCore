//
// Created by eldar on 19.05.2020.
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

float unityPointCloud[720];
float unityIntensities[720];
sensor_msgs::LaserScan message;

void x4Callback(const sensor_msgs::LaserScan x4Cloud)
{
    for (int i = 540; i < 720; i++)
    {
        unityPointCloud[i] = x4Cloud.ranges[i];
        unityIntensities[i] = x4Cloud.intensities[i];
    }
    for (int i = 0; i < 180; i++)
    {
        unityPointCloud[i] = x4Cloud.ranges[i];
        unityIntensities[i] = x4Cloud.intensities[i];
    }
    message.angle_increment = x4Cloud.angle_increment;
    message.angle_max = x4Cloud.angle_max;
    message.angle_min = x4Cloud.angle_min;
    message.header = x4Cloud.header;
//    message.intensities = x4Cloud.intensities;
    message.range_max = x4Cloud.range_max;
    message.range_min = x4Cloud.range_min;
    message.scan_time = x4Cloud.scan_time;
    message.time_increment = x4Cloud.time_increment;
    //std::cout << std::endl;
}

void f4Callback(const sensor_msgs::LaserScan::ConstPtr f4Cloud)
{
    double a = 0, b = 0, c = 0, d = 0;
    for (int i = 0; i < 151; ++i) {
        a += f4Cloud->ranges[i];
    }
    a /= 151;
    for (int i = 151; i < 302; ++i) {
        b += f4Cloud->ranges[i];
    }
    b /= 151;
    for (int i = 302; i < 454; ++i) {
        c += f4Cloud->ranges[i];
    }
    c /= 152;
    for (int i = 454; i < 606; ++i) {
        d += f4Cloud->ranges[i];
    }
    d /= 152;

    ROS_INFO("A= %f; B= %f; C= %f; D= %f.", a, b, c, d);

    /*
    int j = 180;
    for (int i = 540; i < 720; i++)
    {
        unityPointCloud[j] = f4Cloud->ranges[i];
        unityIntensities[j] = f4Cloud->intensities[i];
        j++;
    }
    for (int i = 0; i < 180; i++)
    {
        unityIntensities[j] = f4Cloud->intensities[i];
        j++;
    }*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ydlidar_unity_node");
    ros::NodeHandle nodeHandle;
    const ros::Subscriber f4Subscriber = nodeHandle.subscribe("scanF4", 1000, f4Callback);
    const ros::Subscriber x4Subscriber = nodeHandle.subscribe("scan", 1000, x4Callback);
    const ros::Publisher unityPublisher = nodeHandle.advertise<sensor_msgs::LaserScan>("scanUnity", 1000);
    while (ros::ok())
    {
        message.ranges.clear();
        message.intensities.clear();
        for (int i = 0; i < 719; i++)
        {
            if (i < 0.1 && i > 0) {
                ROS_WARN("Fault detected at %d", i);
            }
            message.ranges.push_back(unityPointCloud[i]);
            message.intensities.push_back(unityIntensities[i]);
        }
        /*for (auto i: message.ranges)
            std::cout << i << ' ';
        std::cout << std::endl << std::endl;*/
        unityPublisher.publish(message);
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}