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
    /*
     * на этом лидаре всего 720 точек.
     * c 270 по 449 это слепая зона лидара
     * с 90 по 269 это левая часть робота
     * с 450 по 629 это правая часть робота
     * с 630 по 718 и с 0 по 89 это передняя часть робота
     * это базовый лидар поэтому массив из F4 надо подгонять под него
     */

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
    //ROS_INFO("F4 cloud: %lu", f4Cloud->ranges.size());

    /*double a = 0, b = 0, c = 0, d = 0;
    for (int i = 270; i < 450; ++i) {
        a += f4Cloud->ranges[i];
    }
    a /= 180;
    for (int i = 90; i < 270; ++i) {
        b += f4Cloud->ranges[i];
    }
    b /= 180;
    for (int i = 450; i < 630; ++i) {
        c += f4Cloud->ranges[i];
    }
    c /= 180;
    for (int i = 630; i < 720; ++i) {
        d += f4Cloud->ranges[i];
    }
    for (int i = 0; i < 90; ++i) {
        d += f4Cloud->ranges[i];
    }
    d /= 180;

    ROS_INFO("A= %f; B= %f; C= %f; D= %f.", a, b, c, d);*/

    //POINT CLOUD VIEWER
    for (int i = 0; i < 789; i++) {
        if (!(i % 8))
            std::cout << std::setfill('0') << std::setw(3) << std::floor(f4Cloud->ranges[i] * 10)/10 << " ";
    }
    std::cout << std::endl;
    //PUT IT TO TEXT EDITOR
    // 001 002 003 004 005 006 007 008 009 010 011 012 013 014 015 016 017 018 019 020 021 022 023 024 025 026 027 028 029 030 031 032 033 034 035 036 037 038 039 040 041 042 043 044 045 046 047 048 049 050 051 052 053 054 055 056 057 058 059 060 061 062 063 064 065 066 067 068 069 070 071 072 073 074 075 076 077 078 079 080 081 082 083 084 085 086 087 088 089 090 091 092 093 094 095 096 097 098 099
    //END POINT CLOUD VIEWER

    /*
     * результаты эксперимента:
     * всего лидар выдаёт 789 точек
     * c 0 по 174 и с 650 по 789 это слепая зона лидара
     * с 175 по 299 это левая часть робота
     * с 300 по 519 это задняя часть робота
     * с 520 по 649 это правая часть робота
     * решено взять следующие диапазоны
     * лево 192 по 282 (90 элементов)
     * право (520+649) / 2 = 584. 584-45=539 по 539+90 = 629 (90 элементов)
     * зад с 319 по 409 (180 элементов)
     */

    int j = 180;
    for (int i = 192; i < 282; i++)
    {
        unityPointCloud[j] = f4Cloud->ranges[i];
        unityIntensities[j] = f4Cloud->intensities[i];
        j++;
    }
    for (int i = 319; i < 499; i++)
    {
        unityPointCloud[j] = f4Cloud->ranges[i];
        unityIntensities[j] = f4Cloud->intensities[i];
        j++;
    }
    for (int i = 539; i < 629; i++)
    {
        unityPointCloud[j] = f4Cloud->ranges[i];
        unityIntensities[j] = f4Cloud->intensities[i];
        j++;
    }
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