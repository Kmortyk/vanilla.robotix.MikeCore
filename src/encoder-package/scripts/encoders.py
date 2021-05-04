#!/usr/bin/env python

# 75 ticks

import rospy
import Jetson.GPIO as GPIO
import os

pin_l_f = 22
pin_r_f = 21
pin_l_b = 24
pin_r_b = 23

value_l_f = 0
value_l_b = 0
value_r_f = 0
value_r_b = 0

ticker_l = 0
ticker_r = 0

direction_l = 0
direction_r = 0

tick21 = 0
tick22 = 0
tick23 = 0
tick24 = 0


def gpio_callback21(channel):
    global tick21, value_r_f, ticker_r, direction_r
    tick21 += 1
    value_r_f = GPIO.input(21)
    if ticker_r is 0:
        direction_r = 1
    ticker_r = 0


def gpio_callback22(channel):
    global tick22, value_l_f, ticker_l, direction_l
    tick22 += 1
    value_l_f = GPIO.input(22)
    if ticker_l is 0:
        direction_l = 1
    ticker_l = 0


def gpio_callback23(channel):
    global tick23, value_r_b, ticker_r, direction_r
    tick23 += 1
    value_r_b = GPIO.input(23)
    if ticker_r is 1:
        direction_r = 0
    ticker_r = 1


def gpio_callback24(channel):
    global tick24, value_l_b, ticker_l, direction_l
    tick24 += 1
    value_l_b = GPIO.input(24)
    if ticker_l is 1:
        direction_l = 0
    ticker_l = 1


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
    GPIO.cleanup({21, 22, 23, 24})


if __name__ == "__main__":
    main()
