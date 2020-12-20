#!/usr/bin/env python

from gpio_jetson_service.srv import gpio_srv, gpio_srvResponse
from gpio_movement import *


def handle_gpio_command(req):
    gpio_current_command = Command(req.command)
    rospy.loginfo("GPIO Command: %s", gpio_current_command.name)
    # TODO GPIO functions returns errors. Need to catch it.
    if gpio_current_command is Command.FULL_STOP:
        result = 2
        result -= gpio_stop(Motor.left)
        result -= gpio_stop(Motor.right)

    elif gpio_current_command is Command.LEFT_STOP:
        result = 1
        result -= gpio_stop(Motor.left)

    elif gpio_current_command is Command.RIGHT_STOP:
        result = 1
        result -= gpio_stop(Motor.right)

    elif gpio_current_command is Command.FORWARD_LOW:
        result = 2
        result -= gpio_move(Motor.left, Direction.forward, Speed.low)
        result -= gpio_move(Motor.right, Direction.forward, Speed.low)

    elif gpio_current_command is Command.FORWARD_MIDDLE:
        result = 2
        result -= gpio_move(Motor.left, Direction.forward, Speed.middle)
        result -= gpio_move(Motor.right, Direction.forward, Speed.middle)

    elif gpio_current_command is Command.FORWARD_FAST:
        result = 2
        result -= gpio_move(Motor.left, Direction.forward, Speed.fast)
        result -= gpio_move(Motor.right, Direction.forward, Speed.fast)

    elif gpio_current_command is Command.BACKWARD_LOW:
        result = 2
        result -= gpio_move(Motor.left, Direction.backward, Speed.low)
        result -= gpio_move(Motor.right, Direction.backward, Speed.low)

    elif gpio_current_command is Command.BACKWARD_MIDDLE:
        result = 2
        result -= gpio_move(Motor.left, Direction.backward, Speed.middle)
        result -= gpio_move(Motor.right, Direction.backward, Speed.middle)

    elif gpio_current_command is Command.BACKWARD_FAST:
        result = 2
        result -= gpio_move(Motor.left, Direction.backward, Speed.fast)
        result -= gpio_move(Motor.right, Direction.backward, Speed.fast)

    elif gpio_current_command is Command.LEFT_FORWARD_LOW:
        result = 1
        result -= gpio_move(Motor.left, Direction.forward, Speed.low)

    elif gpio_current_command is Command.LEFT_FORWARD_MIDDLE:
        result = 2
        result -= gpio_move(Motor.left, Direction.forward, Speed.middle)
        result -= gpio_move(Motor.right, Direction.backward, Speed.middle)

    elif gpio_current_command is Command.LEFT_FORWARD_FAST:
        result = 1
        result -= gpio_move(Motor.left, Direction.forward, Speed.fast)

    elif gpio_current_command is Command.LEFT_BACKWARD_LOW:
        result = 1
        result -= gpio_move(Motor.left, Direction.backward, Speed.low)

    elif gpio_current_command is Command.LEFT_BACKWARD_MIDDLE:
        result = 2
        result -= gpio_move(Motor.left, Direction.backward, Speed.middle)
        result -= gpio_move(Motor.right, Direction.forward, Speed.middle)

    elif gpio_current_command is Command.LEFT_BACKWARD_FAST:
        result = 1
        result -= gpio_move(Motor.left, Direction.backward, Speed.fast)

    elif gpio_current_command is Command.RIGHT_FORWARD_LOW:
        result = 1
        result -= gpio_move(Motor.right, Direction.forward, Speed.low)

    elif gpio_current_command is Command.RIGHT_FORWARD_MIDDLE:
        result = 2
        result -= gpio_move(Motor.right, Direction.forward, Speed.middle)
        result -= gpio_move(Motor.left, Direction.backward, Speed.middle)

    elif gpio_current_command is Command.RIGHT_FORWARD_FAST:
        result = 1
        result -= gpio_move(Motor.right, Direction.forward, Speed.fast)

    elif gpio_current_command is Command.RIGHT_BACKWARD_LOW:
        result = 1
        result -= gpio_move(Motor.right, Direction.backward, Speed.low)

    elif gpio_current_command is Command.RIGHT_BACKWARD_MIDDLE:
        result = 2
        result -= gpio_move(Motor.right, Direction.backward, Speed.middle)
        result -= gpio_move(Motor.left, Direction.forward, Speed.middle)

    elif gpio_current_command is Command.RIGHT_BACKWARD_FAST:
        result = 1
        result -= gpio_move(Motor.right, Direction.backward, Speed.fast)

    else:
        return gpio_srvResponse(0)

    return gpio_srvResponse(not result)


def gpio_jetson_service():
    rospy.init_node('gpio_jetson_service')
    rospy.Service('gpio_jetson_service', gpio_srv, handle_gpio_command)
    init_result = gpio_init()
    if not init_result:
        rospy.logfatal("GPIO Jetson service didn't start!")
        return
    rospy.loginfo("GPIO Jetson Service started")
    rospy.spin()
    gpio_stop(Motor.left)
    gpio_stop(Motor.right)
    if not gpio_deinit():
        rospy.logfatal("GPIO Jetson service didn't finish properly!")
        return
    rospy.loginfo("GPIO Jetson service shutdown successfully")


if __name__ == "__main__":
    gpio_jetson_service()
