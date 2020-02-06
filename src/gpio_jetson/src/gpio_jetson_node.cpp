//
// Created by eldar on 02.02.2020.
//

/* mapping
 * contact number on GPIO interface : pin number in jetson nano driver : pin number in wiring pi (for raspberry PI)
 * 29 : 149 : 21
 * 31 : 200 : 22
 * 33 : 38 : 23
 * 35 : 76 : 24
 * 37 : 12 : 25
 * 36 : 51 : 27
 * 38 : 77 : 28
 * 40 : 78 : 29
*/

#include "ros/ros.h"
#include "std_msgs/String.h"

#define DELAY_TIME 1

#define MOVE_FORWARD "forward"
#define MOVE_BACKWARD "backward"
#define MOVE_LEFT "left"
#define MOVE_RIGHT "right"
#define STOP "stop"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    std::string message = msg->data;
    if (message.find(MOVE_FORWARD) != std::string::npos) {
        ROS_INFO("Moving to: [%s]", msg->data.c_str());
        system("echo 1 > /sys/class/gpio/gpio149/value");
        system("echo 1 > /sys/class/gpio/gpio200/value");
        system("echo 1 > /sys/class/gpio/gpio12/value");
        system("echo 1 > /sys/class/gpio/gpio51/value");
        sleep(DELAY_TIME);
        system("echo 0 > /sys/class/gpio/gpio149/value");
        system("echo 0 > /sys/class/gpio/gpio200/value");
        system("echo 0 > /sys/class/gpio/gpio12/value");
        system("echo 0 > /sys/class/gpio/gpio51/value");
    } else if (message.find(MOVE_BACKWARD) != std::string::npos) {
        ROS_INFO("Moving to: [%s]", msg->data.c_str());
        system("echo 1 > /sys/class/gpio/gpio38/value");
        system("echo 1 > /sys/class/gpio/gpio76/value");
        system("echo 1 > /sys/class/gpio/gpio77/value");
        system("echo 1 > /sys/class/gpio/gpio78/value");
        sleep(DELAY_TIME);
        system("echo 0 > /sys/class/gpio/gpio38/value");
        system("echo 0 > /sys/class/gpio/gpio76/value");
        system("echo 0 > /sys/class/gpio/gpio77/value");
        system("echo 0 > /sys/class/gpio/gpio78/value");
    } else if (message.find(MOVE_LEFT) != std::string::npos) {
        ROS_INFO("Moving to: [%s]", msg->data.c_str());
        system("echo 1 > /sys/class/gpio/gpio200/value");
        sleep(DELAY_TIME);
        system("echo 0 > /sys/class/gpio/gpio200/value");
    } else if (message.find(MOVE_RIGHT) != std::string::npos) {
        ROS_INFO("Moving to: [%s]", msg->data.c_str());
        system("echo 1 > /sys/class/gpio/gpio149/value");
        sleep(DELAY_TIME);
        system("echo 0 > /sys/class/gpio/gpio149/value");
    } else if (message.find(STOP) != std::string::npos) {
        ROS_INFO("Command [%s]", msg->data.c_str());
        system("echo 0 > /sys/class/gpio/gpio149/value");
        system("echo 0 > /sys/class/gpio/gpio200/value");
        system("echo 0 > /sys/class/gpio/gpio12/value");
        system("echo 0 > /sys/class/gpio/gpio51/value");
    }
}

void gpio_init() {
    system("echo 149 > /sys/class/gpio/export");
    system("echo 200 > /sys/class/gpio/export");
    system("echo 38 > /sys/class/gpio/export");
    system("echo 76 > /sys/class/gpio/export");
    system("echo 12 > /sys/class/gpio/export");
    system("echo 51 > /sys/class/gpio/export");
    system("echo 77 > /sys/class/gpio/export");
    system("echo 78 > /sys/class/gpio/export");

    system("echo out > /sys/class/gpio/gpio149/direction");
    system("echo out > /sys/class/gpio/gpio200/direction");
    system("echo out > /sys/class/gpio/gpio38/direction");
    system("echo out > /sys/class/gpio/gpio76/direction");
    system("echo out > /sys/class/gpio/gpio12/direction");
    system("echo out > /sys/class/gpio/gpio51/direction");
    system("echo out > /sys/class/gpio/gpio77/direction");
    system("echo out > /sys/class/gpio/gpio78/direction");
}

void gpio_deinit() {
    system("echo 149 > /sys/class/gpio/unexport");
    system("echo 200 > /sys/class/gpio/unexport");
    system("echo 38 > /sys/class/gpio/unexport");
    system("echo 76 > /sys/class/gpio/unexport");
    system("echo 12 > /sys/class/gpio/unexport");
    system("echo 51 > /sys/class/gpio/unexport");
    system("echo 77 > /sys/class/gpio/unexport");
    system("echo 78 > /sys/class/gpio/unexport");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpio_jetson_node");
    gpio_init();
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("gpio", 1000, chatterCallback);
    ros::spin();
    gpio_deinit();
    return 0;
}