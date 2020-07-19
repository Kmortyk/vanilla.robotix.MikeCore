//
// Created by eldar on 02.04.2020.
//

#include "ros/ros.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cmath>
#include "cartographer_ros_msgs/SubmapQuery.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

ros::ServiceClient submapQueryClient;
tf::TransformListener* transformListener;
tf::StampedTransform transform_bot;

void submapCallback(const cartographer_ros_msgs::SubmapList::ConstPtr& list)
{
    for (auto s : list->submap) {
        ROS_INFO("submap: %d", s.submap_index);
        ROS_INFO("X: %f", s.pose.position.x);
        ROS_INFO("Y: %f", s.pose.position.y);
        ROS_INFO("Z: %f", s.pose.position.z);
        cartographer_ros_msgs::SubmapQuery service;
        service.request.submap_index = s.submap_index;
        service.request.trajectory_id = s.trajectory_id;
        if (submapQueryClient.call(service)) {
            ROS_INFO("Submap version: %d", service.response.submap_version);
            for (const auto& texture : service.response.textures) {
                ROS_INFO("Resolution: %f, width: %d, height: %d", texture.resolution, texture.width, texture.height);
                auto o = texture.slice_pose.orientation;
                auto p = texture.slice_pose.position;
                ROS_INFO("Slice pose: position. X = %f, Y = %f, Z = %f", p.x, p.y, p.z);
                ROS_INFO("Slice pose: orientation. X = %f, Y = %f, Z = %f, W = %f", o.x, o.y, o.z, o.w);
                int i = 0;
                int j = 0;
                for (auto cell : texture.cells) {
                    printf("%d ", cell);
                    i++;
                    j++;
                    if (i >= texture.width) {
                        putchar('\n');
                        i = 0;
                    }
                }
                ROS_INFO("Texture cells: %d", j);
            }
        } else {
            ROS_ERROR("Error of submap query client!");
        }

    }
}

/*void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
    int width = grid->info.width;
    int i = 0;
    auto position = grid->info.origin.position;
    auto orientation = grid->info.origin.orientation;
    for (auto &p : grid->data)
    {
        printf("%d ", p);
        i++;
        if (i >= width) {
            putchar('\n');
            i = 0;
        }
    }
    ROS_INFO("X = %f OX = %f", position.x, orientation.x);
    ROS_INFO("Y = %f OY = %f", position.y, orientation.y);
    ROS_INFO("Z = %f OZ = %f", position.z, orientation.z);
    ROS_INFO("OW = %f", orientation.w);

    try {
        transformListener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
        transformListener->lookupTransform("base_link", "map", ros::Time(0), transform_bot);
    } catch (tf2::LookupException exception) {
        ROS_ERROR("error: %s", exception.what());
        return;
    }
    //ROS_WARN("Wait for transform passed");

    double bot_x = transform_bot.getOrigin().x();
    double bot_y = transform_bot.getOrigin().y();

    double roll, pitch, yaw;
    transform_bot.getBasis().getRPY(roll, pitch, yaw);

    double bot_dir = yaw * 180.0 / M_PI;
    ROS_INFO("bot_x: %f;\nbot_y: %f; bot_dir: %f;", bot_x, bot_y, bot_dir);
}*/

void mapToWorld(double map_x, double map_y, double& pos_x, double& pos_y, const nav_msgs::OccupancyGrid& map)
{
    pos_x = map.info.origin.position.x + (map_x + 0.5) * map.info.resolution;
    pos_y = map.info.origin.position.y + (map_y + 0.5) * map.info.resolution;
}

/*void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
    try {
        transformListener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
        transformListener->lookupTransform("base_link", "map", ros::Time(0), transform_bot);
    } catch (tf2::LookupException exception) {
        ROS_ERROR("error: %s", exception.what());
        return;
    }
    double bot_x = transform_bot.getOrigin().x();
    double bot_y = transform_bot.getOrigin().y();
    double roll, pitch, yaw;
    transform_bot.getBasis().getRPY(roll, pitch, yaw);
    double bot_dir = yaw * 180.0 / M_PI;
    double x, y;
    mapToWorld(bot_x, bot_y, x, y, *grid);
    int i = 0;
    for (auto &p : grid->data)
    {
        printf("%d ", p);
        i++;
        if (i >= grid->info.width) {
            putchar('\n');
            i = 0;
        }
    }
    double origin_x = grid->info.origin.position.x;
    double origin_y = grid->info.origin.position.y;
    ROS_INFO("origin_x: %f; origin_y: %f; bot_x: %f; bot_y: %f; bot_dir: %f;", origin_x, origin_y, bot_x, bot_y, bot_dir);
}*/

tf2_ros::Buffer tfBuffer;
std::string globalFrame, robotBaseFrame;
geometry_msgs::PoseStamped oldPose;

void init(ros::NodeHandle& nodeHandle)
{
    tf2::toMsg(tf2::Transform::getIdentity(), oldPose.pose);
    nodeHandle.param("global_frame", globalFrame, std::string("map"));
    nodeHandle.param("robot_base_frame", robotBaseFrame, std::string("base_link"));
}

bool getRobotPose(geometry_msgs::PoseStamped& globalPose)
{
    tf2::toMsg(tf2::Transform::getIdentity(), globalPose.pose);
    geometry_msgs::PoseStamped robotPose;
    tf2::toMsg(tf2::Transform::getIdentity(), robotPose.pose);
    robotPose.header.frame_id = robotBaseFrame;
    robotPose.header.stamp = ros::Time();
    try
    {
        tfBuffer.transform(robotPose, globalPose, globalFrame);
    }
    catch (tf2::LookupException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    return true;
}

void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
    //int i = 0;
    /*for (auto &p : grid->data)
    {
        printf("%d ", p);
        i++;
        if (i >= grid->info.width) {
            putchar('\n');
            i = 0;
        }
    }*/
    /*geometry_msgs::PoseStamped pose;
    if (getRobotPose(pose))
    {
        double x = pose.pose.position.x,
        y = pose.pose.position.y,
        yaw = tf2::getYaw(pose.pose.orientation);
        ROS_INFO("X = %f; Y= %f; YAW = %f.", x, y, yaw);
    }*/

    try {
        transformListener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
        transformListener->lookupTransform("base_link", "map", ros::Time(0), transform_bot);
    } catch (tf2::LookupException exception) {
        ROS_ERROR("error: %s", exception.what());
        return;
    }

    double bot_x = transform_bot.getOrigin().x();
    double bot_y = transform_bot.getOrigin().y();

    double roll, pitch, yaw;
    transform_bot.getBasis().getRPY(roll, pitch, yaw);

    double bot_dir = yaw * 180.0 / M_PI;

    double i = (bot_y - grid->info.origin.position.y) / grid->info.resolution;
    double j = (bot_x - grid->info.origin.position.x) / grid->info.resolution;

    int i_int = (int) i;
    int j_int = (int) j;

    int k = 0;
    int c = 0;
    for (auto &p : grid->data)
    {
        if (c == i && k == j) {
            printf("\033[0;31mH\033[0m ");
        } else {
            printf("%d ", p);
        }
        k++;
        if (k >= grid->info.width) {
            putchar('\n');
            k = 0;
            c++;
        }
    }

    ROS_INFO("i = %f; j = %f; dir = %f", i, j, bot_dir);
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
    tfBuffer.setUsingDedicatedThread(true);
    //ros::Subscriber subscriber = nodeHandle.subscribe("submap_list", 1000, submapCallback);
    ros::Subscriber subscriber1 = nodeHandle.subscribe("map", 1000, occupancyGridCallback);
    //tf::TransformListener listener(nodeHandle);
    //ros::Timer timer = nodeHandle.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
    //tf::StampedTransform transform_bot;
    transformListener = new tf::TransformListener(nodeHandle);
    init(nodeHandle);

    //submapQueryClient = nodeHandle.serviceClient<cartographer_ros_msgs::SubmapQuery>("submap_query");

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}

