//
// Created by eldar on 09.03.2020.
//

#include "commands.hpp"

const uint8_t FULL_STOP = 0, LEFT_STOP = 1, RIGHT_STOP = 2,
        FORWARD_LOW = 3, FORWARD_MIDDLE = 4, FORWARD_FAST = 5,
        BACKWARD_LOW = 6, BACKWARD_MIDDLE = 7, BACKWARD_FAST = 8,
        LEFT_FORWARD_LOW = 9, LEFT_FORWARD_MIDDLE = 10, LEFT_FORWARD_FAST = 11,
        LEFT_BACKWARD_LOW = 12, LEFT_BACKWARD_MIDDLE = 13, LEFT_BACKWARD_FAST = 14,
        RIGHT_FORWARD_LOW = 15, RIGHT_FORWARD_MIDDLE = 16, RIGHT_FORWARD_FAST = 17,
        RIGHT_BACKWARD_LOW = 18, RIGHT_BACKWARD_MIDDLE = 19, RIGHT_BACKWARD_FAST = 20;

std::string MoveCommands::commandName(uint8_t command) {
    switch (command) {
        case FULL_STOP:
            return "full stop";
        case LEFT_STOP:
            return "left stop";
        case RIGHT_STOP:
            return "right stop";
        case FORWARD_LOW:
            return "forward low";
        case FORWARD_MIDDLE:
            return "forward middle";
        case FORWARD_FAST:
            return "forward fast";
        case BACKWARD_LOW:
            return "backward low";
        case BACKWARD_MIDDLE:
            return "backward middle";
        case BACKWARD_FAST:
            return "backward fast";
        case LEFT_FORWARD_LOW:
            return "left forward low";
        case LEFT_FORWARD_MIDDLE:
            return "left forward middle";
        case LEFT_FORWARD_FAST:
            return "left forward fast";
        case LEFT_BACKWARD_LOW:
            return "left backward low";
        case LEFT_BACKWARD_MIDDLE:
            return "left backward middle";
        case LEFT_BACKWARD_FAST:
            return "left backward fast";
        case RIGHT_FORWARD_LOW:
            return "right forward low";
        case RIGHT_FORWARD_MIDDLE:
            return "right forward middle";
        case RIGHT_FORWARD_FAST:
            return "right forward fast";
        case RIGHT_BACKWARD_LOW:
            return "right backward low";
        case RIGHT_BACKWARD_MIDDLE:
            return "right backward middle";
        case RIGHT_BACKWARD_FAST:
            return "right backward fast";
        default:
            return "unknown";
    }
}
