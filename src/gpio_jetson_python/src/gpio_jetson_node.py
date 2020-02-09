#!/usr/bin/env python3
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)


def listener():
    rospy.init_node('gpio_jetson_node_python', anonymous=True)
    rospy.Subscriber('gpio', String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
