#!/usr/bin/env python

# 75 ticks

import rospy
import Jetson.GPIO as GPIO
import os

angle_tick = 2.4

pin_l_f = 22
pin_r_f = 21
pin_l_b = 24
pin_r_b = 23

value_l_f = 0
value_l_b = 0
value_r_f = 0
value_r_b = 0

status_l = 0
status_r = 0

direction_l = 0
direction_r = 0

angle_l = 0
angle_r = 0

tick21 = 0
tick22 = 0
tick23 = 0
tick24 = 0


def calc_direction_l():
    global value_l_f, value_l_b, direction_l, status_l
    if status_l is 0:
        if value_l_f is 0:
            direction_l = 0
            status_l = 3
        if value_l_f is 1:
            direction_l = 1
            status_l = 1
    elif status_l is 1:
        if value_l_b is 0:
            direction_l = 0
            status_l = 0
        if value_l_b is 1:
            direction_l = 1
            status_l = 2
    elif status_l is 2:
        if value_l_f is 1:
            direction_l = 0
            status_l = 1
        if value_l_f is 0:
            direction_l = 1
            status_l = 3
    elif status_l is 3:
        if value_l_f is 1:
            direction_l = 0
            status_l = 2
        if value_l_f is 0:
            direction_l = 1
            status_l = 0


def calc_direction_r():
    global value_r_f, value_r_b, direction_r, status_r
    if status_r is 0:
        if value_r_f is 0:
            direction_r = 1
            status_r = 3
        if value_r_f is 1:
            direction_r = 0
            status_r = 1
    elif status_r is 1:
        if value_r_b is 0:
            direction_r = 1
            status_r = 0
        if value_r_b is 1:
            direction_r = 0
            status_r = 2
    elif status_r is 2:
        if value_r_f is 1:
            direction_r = 1
            status_r = 1
        if value_r_f is 0:
            direction_r = 0
            status_r = 3
    elif status_r is 3:
        if value_r_f is 1:
            direction_r = 1
            status_r = 2
        if value_r_f is 0:
            direction_r = 0
            status_r = 0


def gpio_callback21(channel):
    global tick21, value_r_f, status_r, direction_r, angle_r
    tick21 += 1
    value_r_f = GPIO.input(21)
    calc_direction_r()
    if direction_r is 0:
        angle_r += angle_tick
    else:
        angle_r -= angle_tick


def gpio_callback22(channel):
    global tick22, value_l_f, status_l, direction_l, angle_l
    tick22 += 1
    value_l_f = GPIO.input(22)
    calc_direction_l()
    if direction_l is 0:
        angle_l += angle_tick
    else:
        angle_l -= angle_tick


def gpio_callback23(channel):
    global tick23, value_r_b, status_r, direction_r, angle_r
    tick23 += 1
    value_r_b = GPIO.input(23)
    calc_direction_r()
    if direction_r is 0:
        angle_r += angle_tick
    else:
        angle_r -= angle_tick


def gpio_callback24(channel):
    global tick24, value_l_b, status_l, direction_l, angle_l
    tick24 += 1
    value_l_b = GPIO.input(24)
    calc_direction_l()
    if direction_l is 0:
        angle_l += angle_tick
    else:
        angle_l -= angle_tick


def calc_status():
    global value_l_f, value_l_b, value_r_f, value_r_b, status_l, status_r
    if value_l_f is 0 and value_l_b is 0:
        status_l = 0
    elif value_l_f is 1 and value_l_b is 0:
        status_l = 1
    elif value_l_f is 1 and value_l_b is 1:
        status_l = 2
    elif value_l_f is 0 and value_l_b is 1:
        status_l = 3

    if value_r_f is 0 and value_r_b is 0:
        status_r = 0
    elif value_r_f is 1 and value_r_b is 0:
        status_r = 1
    elif value_r_f is 1 and value_r_b is 1:
        status_r = 2
    elif value_r_f is 0 and value_r_b is 1:
        status_r = 3


def main():
    global value_l_f, value_l_b, value_r_f, value_r_b
    rospy.init_node('encoder')
    rate = rospy.Rate(10)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup({21, 22, 23, 24}, GPIO.IN)
    value_l_f = GPIO.input(pin_l_f)
    value_l_b = GPIO.input(pin_l_b)
    value_r_f = GPIO.input(pin_r_f)
    value_r_b = GPIO.input(pin_r_b)
    calc_status()
    GPIO.add_event_detect(21, GPIO.BOTH, callback=gpio_callback21)
    GPIO.add_event_detect(22, GPIO.BOTH, callback=gpio_callback22)
    GPIO.add_event_detect(23, GPIO.BOTH, callback=gpio_callback23)
    GPIO.add_event_detect(24, GPIO.BOTH, callback=gpio_callback24)
    while not rospy.is_shutdown():
        os.system('clear')
        print("21 ", tick21)
        print("22 ", tick22)
        print("23 ", tick23)
        print("24 ", tick24)
        print(value_l_f, " ", value_l_b, " ", value_r_f, " ", value_r_b)
        print(direction_l, direction_r)
        print(angle_l, angle_r)
    GPIO.cleanup({21, 22, 23, 24})


if __name__ == "__main__":
    main()
