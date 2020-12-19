# Xavier NX new GPIO
# 35 - right backward
# 37 - right forward
# 33 - right pwm
# 32 - left pwm
# 38 - left forward
# 40 - left backward

import rospy
import Jetson.GPIO as GPIO
from gpio_constants import *

gpio_initialized = 0
# noinspection PyTypeChecker
pwm_left = None # type: GPIO.PWM
# noinspection PyTypeChecker
pwm_right = None # type: GPIO.PWM


def gpio_init():
    global pwm_left, pwm_right, gpio_initialized
    if gpio_initialized:
        rospy.logerr("GPIO Already init!")
        return 0
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(gpio_pins, GPIO.OUT, initial=GPIO.LOW)
    pwm_left = GPIO.PWM(32, 20)
    pwm_right = GPIO.PWM(33, 20)
    gpio_initialized = 1
    return gpio_initialized


def gpio_deinit():
    global pwm_left, pwm_right, gpio_initialized
    if not gpio_initialized:
        rospy.logerr("GPIO doesn't init!")
        return 0
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
    gpio_initialized = 0
    return 1


def gpio_stop(motor):
    return 1


def gpio_move(motor, direction, speed):
    return 0