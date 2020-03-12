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
            case 'q':
                move_command = MoveCommands::FULL_STOP;
                break;
            case 'w':
                move_command = MoveCommands::LEFT_STOP;
                break;
            case 'e':
                move_command = MoveCommands::RIGHT_STOP;
                break;
            case 'r':
                move_command = MoveCommands::FORWARD_LOW;
                break;
            case 't':
                move_command = MoveCommands::FORWARD_MIDDLE;
                break;
            case 'y':
                move_command = MoveCommands::FORWARD_FAST;
                break;
            case 'u':
                move_command = MoveCommands::BACKWARD_LOW;
                break;
            case 'i':
                move_command = MoveCommands::BACKWARD_MIDDLE;
                break;
            case 'o':
                move_command = MoveCommands::BACKWARD_FAST;
                break;
            case 'p':
                move_command = MoveCommands::LEFT_FORWARD_LOW;
                break;
            case '[':
                move_command = MoveCommands::LEFT_FORWARD_MIDDLE;
                break;
            case ']':
                move_command = MoveCommands::LEFT_FORWARD_FAST;
                break;
            case 'a':
                move_command = MoveCommands::LEFT_BACKWARD_LOW;
                break;
            case 's':
                move_command = MoveCommands::LEFT_BACKWARD_MIDDLE;
                break;
            case 'd':
                move_command = MoveCommands::LEFT_BACKWARD_FAST;
                break;
            case 'f':
                move_command = MoveCommands::RIGHT_FORWARD_LOW;
                break;
            case 'g':
                move_command = MoveCommands::RIGHT_FORWARD_MIDDLE;
                break;
            case 'h':
                move_command = MoveCommands::RIGHT_FORWARD_FAST;
                break;
            case 'j':
                move_command = MoveCommands::RIGHT_BACKWARD_LOW;
                break;
            case 'k':
                move_command = MoveCommands::RIGHT_BACKWARD_MIDDLE;
                break;
            case 'l':
                move_command = MoveCommands::RIGHT_BACKWARD_FAST;
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