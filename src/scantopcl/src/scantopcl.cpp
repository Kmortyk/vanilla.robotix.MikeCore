//
// Created by eldar on 9/29/20.
//

#include "scantopcl.h"

My_Filter::My_Filter(){
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &My_Filter::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/base_scan", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    if(!tfListener_.waitForTransform(
            scan->header.frame_id,
            "/base_link",
            ros::Time(),
            ros::Duration(1.0))){
        return;
    }
    try {
        projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    } catch(tf2::ExtrapolationException &e) {
        std::cout << e.what() << std::endl;
    }
    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scantopcl");

    My_Filter filter;

    ros::spin();

    return 0;
}