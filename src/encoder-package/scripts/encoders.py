#!/usr/bin/env python

import rospy
import Jetson.GPIO as GPIO
import os

tick21 = 0
tick22 = 0
tick23 = 0
tick24 = 0


def gpio_callback21(channel):
    global tick21
    tick21 += 1
        

def gpio_callback22(channel):
    global tick22
    tick22 += 1

def gpio_callback23(channel):
    global tick23
    tick23 += 1

def gpio_callback24(channel):
    global tick24
    tick24 += 1


# def publish_angle():
#     print("")


def main():
    rospy.init_node('encoder')
    rate = rospy.Rate(10)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup({21, 22, 23, 24}, GPIO.IN)
    GPIO.add_event_detect(21, GPIO.FALLING)
    GPIO.add_event_detect(22, GPIO.FALLING)
    GPIO.add_event_detect(23, GPIO.FALLING)
    GPIO.add_event_detect(24, GPIO.FALLING)
    GPIO.add_event_callback(21, gpio_callback21)
    GPIO.add_event_callback(22, gpio_callback22)
    GPIO.add_event_callback(23, gpio_callback23)
    GPIO.add_event_callback(24, gpio_callback24)
    while not rospy.is_shutdown():
	os.system('clear')
        print("21 ", tick21)
        print("22 ", tick22)
        print("23 ", tick23)
        print("24 ", tick24)
        # publish_angle()
        # rate.sleep()
    GPIO.cleanup({21, 22, 23, 24})


if __name__ == "__main__":
    main()
