#!/usr/bin/env python

import rospy
import Jetson.GPIO as GPIO

def gpio_callback(channel):
    print(channel)


def publish_angle():
    print("")


def main():
    rospy.init_node('encoder')
    rate = rospy.Rate(10)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup({21, 22, 23, 24}, GPIO.IN)
    GPIO.add_event_detect(21, GPIO.BOTH)
    GPIO.add_event_detect(22, GPIO.BOTH)
    GPIO.add_event_detect(23, GPIO.BOTH)
    GPIO.add_event_detect(24, GPIO.BOTH)
    GPIO.add_event_callback(21, gpio_callback)
    GPIO.add_event_callback(22, gpio_callback)
    GPIO.add_event_callback(23, gpio_callback)
    GPIO.add_event_callback(24, gpio_callback)
    while not rospy.is_shutdown():
        publish_angle()
        rate.sleep()
    GPIO.cleanup({21, 22, 23, 24})


if __name__ == "__main__":
    main()