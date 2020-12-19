#!/usr/bin/env python

from gpio_jetson_service.srv import gpio_srv, gpio_srvResponse
import rospy
from gpio_movement import *
import gpio_constants


def handle_gpio_command(req):
    rospy.loginfo("GPIO Command: %s", gpio_constants.Command(req.command).title)
    return gpio_srvResponse(1)


def gpio_jetson_service():
    rospy.init_node('gpio_jetson_service')
    s = rospy.Service('gpio_jetson_service', gpio_srv, handle_gpio_command)
    init_result = gpio_init()
    if not init_result:
        rospy.logfatal("GPIO Jetson service didn't start!")
        return
    rospy.loginfo("GPIO Jetson Service started")
    rospy.spin()
    gpio_stop(gpio_constants.Motor.left)
    gpio_stop(gpio_constants.Motor.right)
    if not gpio_deinit():
        rospy.logfatal("GPIO Jetson service didn't finish properly!")
        return
    rospy.loginfo("GPIO Jetson service shutdown successfully")


if __name__ == "__main__":
    gpio_jetson_service()
