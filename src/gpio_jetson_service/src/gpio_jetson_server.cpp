//
// Created by eldar on 09.03.2020.
//

#include "ros/ros.h"
#include "gpio_jetson_service/gpio_srv.h"
#include "commands.hpp"
#include "movement.hpp"

using namespace GPIO_Movement;

bool serviceHandler(gpio_jetson_service::gpio_srv::Request &request, gpio_jetson_service::gpio_srv::Response &response)
{
    ROS_INFO("GPIO Command: %s", MoveCommands::commandName(request.command).c_str());
    switch (request.command) {
        case MoveCommands::FULL_STOP:
            GPIO_Movement::stop(Motor::LEFT);
            GPIO_Movement::stop(Motor::RIGHT);
            break;
        case MoveCommands::LEFT_STOP:
            GPIO_Movement::stop(Motor::LEFT);
            break;
        case MoveCommands::RIGHT_STOP:
            GPIO_Movement::stop(Motor::RIGHT);
            break;
        case MoveCommands::FORWARD_LOW:
            GPIO_Movement::move(Motor::LEFT, Direction::FORWARD, Speed::LOW);
            GPIO_Movement::move(Motor::RIGHT, Direction::FORWARD, Speed::LOW);
            break;
        case MoveCommands::FORWARD_MIDDLE:
            GPIO_Movement::move(Motor::LEFT, Direction::FORWARD, Speed::MIDDLE);
            GPIO_Movement::move(Motor::RIGHT, Direction::FORWARD, Speed::MIDDLE);
            break;
        case MoveCommands::FORWARD_FAST:
            GPIO_Movement::move(Motor::LEFT, Direction::FORWARD, Speed::FAST);
            GPIO_Movement::move(Motor::RIGHT, Direction::FORWARD, Speed::FAST);
            break;
        case MoveCommands::BACKWARD_LOW:
            GPIO_Movement::move(Motor::LEFT, Direction::BACKWARD, Speed::LOW);
            GPIO_Movement::move(Motor::RIGHT, Direction::BACKWARD, Speed::LOW);
            break;
        case MoveCommands::BACKWARD_MIDDLE:
            GPIO_Movement::move(Motor::LEFT, Direction::BACKWARD, Speed::MIDDLE);
            GPIO_Movement::move(Motor::RIGHT, Direction::BACKWARD, Speed::MIDDLE);
            break;
        case MoveCommands::BACKWARD_FAST:
            GPIO_Movement::move(Motor::LEFT, Direction::BACKWARD, Speed::FAST);
            GPIO_Movement::move(Motor::RIGHT, Direction::BACKWARD, Speed::FAST);
            break;
        case MoveCommands::LEFT_FORWARD_LOW:
            GPIO_Movement::move(Motor::LEFT, Direction::FORWARD, Speed::LOW);
            GPIO_Movement::move(Motor::RIGHT, Direction::BACKWARD, Speed::LOW);
            break;
        case MoveCommands::LEFT_FORWARD_MIDDLE:
            GPIO_Movement::move(Motor::LEFT, Direction::FORWARD, Speed::MIDDLE);
            GPIO_Movement::move(Motor::RIGHT, Direction::BACKWARD, Speed::MIDDLE);
            break;
        case MoveCommands::LEFT_FORWARD_FAST:
            GPIO_Movement::move(Motor::LEFT, Direction::FORWARD, Speed::FAST);
            GPIO_Movement::move(Motor::RIGHT, Direction::BACKWARD, Speed::FAST);
            break;
        case MoveCommands::LEFT_BACKWARD_LOW:
            GPIO_Movement::move(Motor::LEFT, Direction::BACKWARD, Speed::LOW);
            GPIO_Movement::move(Motor::RIGHT, Direction::FORWARD, Speed::LOW);
            break;
        case MoveCommands::LEFT_BACKWARD_MIDDLE:
            GPIO_Movement::move(Motor::LEFT, Direction::BACKWARD, Speed::MIDDLE);
            GPIO_Movement::move(Motor::RIGHT, Direction::FORWARD, Speed::MIDDLE);
            break;
        case MoveCommands::LEFT_BACKWARD_FAST:
            GPIO_Movement::move(Motor::LEFT, Direction::BACKWARD, Speed::FAST);
            GPIO_Movement::move(Motor::RIGHT, Direction::FORWARD, Speed::FAST);
            break;
        case MoveCommands::RIGHT_FORWARD_LOW:
            GPIO_Movement::move(Motor::RIGHT, Direction::FORWARD, Speed::LOW);
            GPIO_Movement::move(Motor::LEFT, Direction::BACKWARD, Speed::LOW);
            break;
        case MoveCommands::RIGHT_FORWARD_MIDDLE:
            GPIO_Movement::move(Motor::RIGHT, Direction::FORWARD, Speed::MIDDLE);
            GPIO_Movement::move(Motor::LEFT, Direction::BACKWARD, Speed::MIDDLE);
            break;
        case MoveCommands::RIGHT_FORWARD_FAST:
            GPIO_Movement::move(Motor::RIGHT, Direction::FORWARD, Speed::FAST);
            GPIO_Movement::move(Motor::LEFT, Direction::BACKWARD, Speed::FAST);
            break;
        case MoveCommands::RIGHT_BACKWARD_LOW:
            GPIO_Movement::move(Motor::RIGHT, Direction::BACKWARD, Speed::LOW);
            GPIO_Movement::move(Motor::LEFT, Direction::FORWARD, Speed::LOW);
            break;
        case MoveCommands::RIGHT_BACKWARD_MIDDLE:
            GPIO_Movement::move(Motor::RIGHT, Direction::BACKWARD, Speed::MIDDLE);
            GPIO_Movement::move(Motor::LEFT, Direction::FORWARD, Speed::MIDDLE);
            break;
        case MoveCommands::RIGHT_BACKWARD_FAST:
            GPIO_Movement::move(Motor::RIGHT, Direction::BACKWARD, Speed::FAST);
            GPIO_Movement::move(Motor::LEFT, Direction::FORWARD, Speed::FAST);
            break;
        default:
            return false;
    }
    return response.success = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpio_jetson_service");
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service = nodeHandle.advertiseService("gpio_jetson_service", serviceHandler);
    bool result = GPIO_Movement::init();
    ROS_INFO("Init of GPIO: %s", result ? "success" : "failed");
    if (!result) {
        ROS_FATAL("GPIO service didn't start!");
        ros::requestShutdown();
        return EXIT_FAILURE;
    }
    ROS_INFO("GPIO service ready to work");
    while (ros::ok())
    {
        ros::spinOnce();
    }
    GPIO_Movement::stop(Motor::LEFT);
    GPIO_Movement::stop(Motor::RIGHT);
    result = GPIO_Movement::de_init();
    ROS_INFO("De-init of GPIO: %s", result ? "success" : "failed");
    if (!result) {
        ROS_FATAL("GPIO service didn't finish properly!");
        ros::requestShutdown();
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}