#!/usr/bin/env python
import rospy
import tf
import pyautogui
import numpy
import os
from sensor_msgs.msg import LaserScan
from pathlib import Path

screenshot_region = (400, 100, 950, 650)
transforms = None
rotations = None
times = None
lasers = None
time_now = 0
transform = []
rotation = []

screenshots_folder = Path(os.path.realpath(__file__))\
                         .parent.parent.as_posix() + os.sep + 'data' + os.sep + 'temp_screenshots' + os.sep


def create_dir_securely(path):
    try:
        os.mkdir(path)
    except OSError:
        if not os.path.isdir(path):
            raise


def take_rviz_screenshot():
    pyautogui.screenshot(screenshots_folder + str(time_now) + '.jpg', screenshot_region)


def transform_listen():
    global transform, rotation
    try:
        (transform, rotation) = transform_listener.lookupTransform('base_link',
                                                                   'map',
                                                                   rospy.Time(0))
        rospy.logdebug("", transform, rotation)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Exception when transform listen")


def save_data(laser_data):
    global transforms, rotations, times, lasers
    transform_data = numpy.array([transform[0], transform[1]])
    rotation_data = numpy.array(rotations)
    time_data = [time_now]
    if transforms is None:
        transforms = numpy.array(transform_data)
        rotations = numpy.array(rotation_data)
        times = numpy.array(time_data)
        lasers = numpy.array(laser_data)
    else:
        numpy.append(transforms, transform_data)
        numpy.append(rotations, rotation_data)
        numpy.append(times, time_data)
        numpy.append(lasers, laser_data)


def laser_callback(data):
    global time_now
    time_now = rospy.Time.now().to_nsec()
    transform_listen()
    take_rviz_screenshot()
    rospy.logdebug(data)
    if transform is not None:
        save_data(data)
    rospy.loginfo("Frame passed")


def finalize_data():
    name_situation = raw_input("Enter name of this situation: ")
    situation_folder = Path(os.path.realpath(__file__)) \
        .parent.parent.as_posix() + os.sep + 'data' + os.sep + 'situations' + os.sep + name_situation + os.sep
    if os.path.isdir(situation_folder):
        rospy.logerr("Situation already exists!")
        finalize_data()
        return
    os.mkdir(situation_folder)
    os.mkdir(situation_folder + os.sep + 'screenshots' + os.sep)
    numpy.savez(situation_folder + 'data.npz', transforms, rotations, times, lasers)
    (_, _, filenames) = next(os.walk(screenshots_folder))
    for filename in filenames:
        os.rename(screenshots_folder + filename, situation_folder + 'screenshots' + os.sep + filename)


if __name__ == '__main__':
    create_dir_securely(Path(os.path.realpath(__file__)).parent.parent.as_posix() + os.sep + 'data')
    create_dir_securely(Path(os.path.realpath(__file__)).parent.parent.as_posix() + os.sep + 'data'
                        + os.sep + 'temp_screenshots' + os.sep)
    rospy.init_node('exporter')
    transform_listener = tf.TransformListener()
    rospy.Subscriber('scan', LaserScan, laser_callback)
    rospy.spin()
    finalize_data()
