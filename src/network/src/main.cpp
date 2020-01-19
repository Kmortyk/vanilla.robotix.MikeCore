//
// Created by kmortyk on 19.01.2020.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "network.h"

int main(int argc, char **argv) {

    ros::init(argc, argvm "mike_network");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("network_messages", 1000);
    ros::Rate loop_rate(10);

    // init udp server
    UdpNetwork udp_net(1031, 1031);

    // start network loop
    while(ros::ok()) {
        std_msgs::String msg;
        msg.data = udp_net.recv();
        ROS_INFO("%s", msg.data.c_str());
        // send data
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
