//
// Created by eldar on 09.03.2020.
//

#include "movement.hpp"
#include <wiringPi.h>

bool initialized = false;
std::vector<int> all_pins;

bool GPIO_Movement::init()
{
    if (initialized) {
        ROS_ERROR("GPIO already init!");
        return false;
    }
    int result = 0;
    //all_pins = {149, 200, 38, 76, 12, 51, 77, 78};
    all_pins = {22, 23, 24, 25};
    putenv ("WIRINGPI_GPIOMEM=1") ;
    wiringPiSetup();
    result += pin_export(all_pins);
    if (!result)
        result += pin_direction(all_pins);

    //If result is zero then init is successfully!
    return initialized = !result;
}

bool GPIO_Movement::de_init()
{
    if (!initialized) {
        ROS_ERROR("GPIO doesn't init!");
        return false;
    }
    int result = pin_unexport(all_pins);
    if (result)
        return false;
    else {
        initialized = false;
        return true;
    }
}

bool GPIO_Movement::stop(Motor motor)
{
    if (!initialized) {
        ROS_ERROR("GPIO doesn't init!");
        return false;
    }
    int result;
    std::vector<int> stop_pins;
    switch (motor)
    {
        case LEFT:
            //stop_pins = {78, 76, 51, 200};
            stop_pins = {107, 10};
            result = pin_value(stop_pins, false);
            break;
        case RIGHT:
            //stop_pins = {77, 38, 12, 149};
            stop_pins = {9, 8};
            result = pin_value(stop_pins, false);
            break;
        default:
            return false;
    }
    return !result;
}

bool GPIO_Movement::move(Motor motor, Direction direction, Speed speed)
{
    int result = 0;
    if (!initialized) {
        ROS_ERROR("GPIO doesn't init!");
        return false;
    }
    switch (motor) {
        case LEFT:
            switch (direction) {
                case FORWARD:
                    switch (speed) {
                        case LOW:
                            //result = pin_value(51, true);
                            //break;
                        case MIDDLE:
                            //result = pin_value(200, true);
                            //break;
                        case FAST:
                            //result += pin_value(51, true);
                            //result += pin_value(200, true);
                            result += pin_value(25, true);
                            break;
                        default:
                            return false;
                    }
                    break;
                case BACKWARD:
                    switch (speed) {
                        case LOW:
                            //result = pin_value(76, true);
                            //break;
                        case MIDDLE:
                            //result = pin_value(78, true);
                            //break;
                        case FAST:
                            //result += pin_value(76, true);
                            //result += pin_value(78, true);
                            result += pin_value(24, true);
                            break;
                        default:
                            return false;
                    }
                    break;
                default:
                    return false;
            }
            break;
        case RIGHT:
            switch (direction) {
                case FORWARD:
                    switch (speed) {
                        case LOW:
                            //result = pin_value(12, true);
                            //break;
                        case MIDDLE:
                            //result = pin_value(149, true);
                            //break;
                        case FAST:
                            //result += pin_value(12, true);
                            //result += pin_value(149, true);
                            result += pin_value(23, true);
                            break;
                        default:
                            return false;
                    }
                    break;
                case BACKWARD:
                    switch (speed) {
                        case LOW:
                            //result = pin_value(77, true);
                            //break;
                        case MIDDLE:
                            //result = pin_value(38, true);
                            //break;
                        case FAST:
                            //result += pin_value(77, true);
                            //result += pin_value(38, true);
                            result += pin_value(22, true);
                            break;
                        default:
                            return false;
                    }
                    break;
                default:
                    return false;
            }
            break;
        default:
            return false;
    }
    return !result;
}

int GPIO_Movement::pin_export(int pin)
{
    std::string command = "echo " + std::to_string(pin) + " > /sys/class/gpio/export";
    return system(command.c_str());
}

int GPIO_Movement::pin_export(std::vector<int> &pins)
{
    /*int result = 0;
    for (int pin : pins) {
        std::string command = "echo " + std::to_string(pin) + " > /sys/class/gpio/export";
        result += system(command.c_str());
    }
    return result;*/
    return 0;
}

int GPIO_Movement::pin_unexport(int pin)
{
    std::string command = "echo " + std::to_string(pin) + " > /sys/class/gpio/unexport";
    return system(command.c_str());
}

int GPIO_Movement::pin_unexport(std::vector<int> &pins)
{
    int result = 0;
    for (int pin : pins)
    {
        std::string command = "echo " + std::to_string(pin) + " > /sys/class/gpio/unexport";
        result += system(command.c_str());
    }
    return result;
}

int GPIO_Movement::pin_direction(int pin, Pin_Direction pin_direction)
{
    std::string direction = pin_direction == IN ? "in" : "out";
    std::string command = "echo " + direction + " > /sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    return system(command.c_str());
}

int GPIO_Movement::pin_direction(std::vector<int> &pins, Pin_Direction pin_direction)
{
    /*int result = 0;
    for (int pin : pins)
    {
        std::string direction = pin_direction == IN ? "in" : "out";
        std::string command = "echo " + direction + " > /sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
        result += system(command.c_str());
    }
    return result;*/
    for (int pin : pins)
    {
        pinMode (pin, OUTPUT) ;
    }
}


int GPIO_Movement::pin_value(int pin, bool value)
{
    /*int int_value = value ? 1 : 0;
    std::string command = "echo " + std::to_string(int_value) + " > /sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    return system(command.c_str());*/

    digitalWrite (pin, value) ;
}

int GPIO_Movement::pin_value(std::vector<int> &pins, bool value)
{
    /*int result = 0;
    int int_value = value ? 1 : 0;
    for (int pin : pins)
    {
        std::string command = "echo " + std::to_string(int_value) + " > /sys/class/gpio/gpio" + std::to_string(pin) + "/value";
        result += system(command.c_str());
    }
    return result;*/
    for (int pin : pins)
    {
        digitalWrite(pin, value);
    }
}