//
// Created by eldar on 12.03.2020.
//

#include "ros/ros.h"
#include "gpio_jetson_service/gpio_srv.h"
#include "commands.hpp"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "gpio_manual_control");
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient<gpio_jetson_service::gpio_srv>("gpio_jetson_service");
    while (ros::ok())
    {
        gpio_jetson_service::gpio_srv service;
        char user_input;
        std::cin >> user_input;
        uint8_t move_command = -1;
        switch (user_input)
        {
            case 'w':
                move_command = MoveCommands::FORWARD_FAST;
                break;
            case 'a':
                move_command = MoveCommands::LEFT_FORWARD_FAST;
                break;
            case 's':
                move_command = MoveCommands::BACKWARD_FAST;
                break;
            case 'd':
                move_command = MoveCommands::RIGHT_FORWARD_FAST;
                break;
            default:
                move_command = MoveCommands::FULL_STOP;
        }
        service.request.command = move_command;
        if (client.call(service))
        {
            ROS_INFO("Response from client: %d", service.response.success);
        } else
        {
            ROS_ERROR("Failed to call gpio service!");
        }
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}