#!/usr/bin/env python3
import rospy
import Jetson.GPIO as GPIO
import time
from std_msgs.msg import String

DELAY_TIME = 0.5

MOVE_FORWARD = "forward"
MOVE_BACKWARD = "backward"
MOVE_LEFT = "left"
MOVE_RIGHT = "right"
STOP = "stop"


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    if MOVE_FORWARD in data.data:
        print("Forward")
        pins = [149, 200, 12, 51]
        GPIO.output(pins, GPIO.HIGH)
        time.sleep(DELAY_TIME)
        GPIO.output(pins, GPIO.LOW)
    if MOVE_BACKWARD in data.data:
        print("Backward")
        pins = [38, 76, 77, 78]
        GPIO.output(pins, GPIO.HIGH)
        time.sleep(DELAY_TIME)
        GPIO.output(pins, GPIO.LOW)
    if MOVE_LEFT in data.data:
        print("Left")
        pins = [200]
        GPIO.output(pins, GPIO.HIGH)
        time.sleep(DELAY_TIME)
        GPIO.output(pins, GPIO.LOW)
    if MOVE_RIGHT in data.data:
        print("Right")
        pins = [149]
        GPIO.output(pins, GPIO.HIGH)
        time.sleep(DELAY_TIME)
        GPIO.output(pins, GPIO.LOW)
    if STOP in data.data:
        print("Stop")
        pins = [149, 200, 38, 76, 12, 51, 77, 78]
        GPIO.output(pins, GPIO.LOW)


def gpio_init():
    GPIO.setmode(GPIO.BOARD)
    pins = [149, 200, 38, 76, 12, 51, 77, 78]
    GPIO.setup(pins, GPIO.output, initial=GPIO.LOW)


def listener():
    rospy.init_node('gpio_jetson_node_python', anonymous=True)
    rospy.Subscriber('gpio', String, callback)
    rospy.spin()


if __name__ == '__main__':
    gpio_init()
    listener()
    GPIO.cleanup()
