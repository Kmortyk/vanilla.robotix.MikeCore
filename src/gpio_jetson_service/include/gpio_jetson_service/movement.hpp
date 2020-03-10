//
// Created by eldar on 09.03.2020.
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

#ifndef SRC_MOVEMENT_HPP
#define SRC_MOVEMENT_HPP

#include <iostream>
#include <vector>
#include "ros/ros.h"

namespace GPIO_Movement {

    enum Motor {
        LEFT, RIGHT
    };

    enum Direction {
        FORWARD, BACKWARD
    };

    enum Speed {
        LOW, MIDDLE, FAST
    };

    enum Pin_Direction {
        IN, OUT
    };

    bool init();
    bool de_init();
    bool move(Motor, Direction, Speed);
    bool stop(Motor);

    int pin_export(int pin);
    int pin_export(std::vector<int> &pins);
    int pin_unexport(int pin);
    int pin_unexport(std::vector<int> &pins);
    int pin_direction(int pin, Pin_Direction pin_direction = OUT);
    int pin_direction(std::vector<int> &pins, Pin_Direction pin_direction = OUT);
    int pin_value(int pin, bool value);
    int pin_value(std::vector<int> &pins, bool value);
}

#endif //SRC_MOVEMENT_HPP
