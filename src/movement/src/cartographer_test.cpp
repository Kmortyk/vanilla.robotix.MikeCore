//
// Created by eldar on 02.04.2020.
//

#include "ros/ros.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include <math.h>

void submapCallback(const cartographer_ros_msgs::SubmapList::ConstPtr& list)
{
    //ROS_INFO("%s", list->header.frame_id.c_str());
}

void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
    for (auto &p : grid->data)
    {
        //ROS_INFO("%d", p);
    }
}

void transformPoint(const tf::TransformListener& listener) {
    geometry_msgs::PointStamped laser_point;

    //laser_point.header.frame_id = "base_laser";
    laser_point.header.frame_id = "base_link";

    laser_point.header.stamp = ros::Time();
    laser_point.point.x = 0.0;
    laser_point.point.y = 0.0;
    laser_point.point.z = 0.0;

    try {
        geometry_msgs::PointStamped base_point;
        //listener.transformPoint("base_link", laser_point, base_point);
        listener.transformPoint("map", laser_point, base_point);

        ROS_INFO("base_link: (%.2f, %.2f, %.2f)",
                 base_point.point.x, base_point.point.y, base_point.point.z);
    } catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"map\": %s", ex.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartographer_test");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("submap_list", 1000, submapCallback);
    ros::Subscriber subscriber1 = nodeHandle.subscribe("map", 1000, occupancyGridCallback);
    tf::TransformListener listener(nodeHandle);
    //ros::Timer timer = nodeHandle.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
    tf::StampedTransform transform_bot;
    while (ros::ok())
    {
        listener.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("base_link", "map", ros::Time(0), transform_bot);

        double bot_x = transform_bot.getOrigin().x();
        double bot_y = transform_bot.getOrigin().y();

        double roll, pitch, yaw;
        transform_bot.getBasis().getRPY(roll, pitch, yaw);

        double bot_dir = yaw * 180.0 / M_PI;

        ROS_INFO("Bot X: %f", bot_x);
        ROS_INFO("Bot Y: %f", bot_y);
        ROS_INFO("Bot Z: %f", bot_dir);
        sleep(1);
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}

