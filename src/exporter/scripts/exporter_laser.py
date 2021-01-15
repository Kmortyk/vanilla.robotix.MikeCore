#!/usr/bin/env python
import rospy
import tf
import pyautogui

from sensor_msgs.msg import LaserScan


def take_rviz_screenshot():
    pyautogui.screenshot('screenshots/screen.jpg', region=(0, 0, 300, 300))


def transform_listen():
    try:
        (tranform, rotation) = transform_listener.lookupTransform('base_link',
                                                                  'map',
                                                                  rospy.Time(0))
        rospy.loginfo("%f %f", tranform, rotation)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Exception when transform listen")


def laser_callback(data):
    transform_listen()
    take_rviz_screenshot()
    rospy.loginfo(data)


if __name__ == '__main__':
    rospy.init_node('exporter')
    transform_listener = tf.TransformListener()
    rospy.Subscriber('scan', LaserScan, laser_callback)
    rospy.spin()
