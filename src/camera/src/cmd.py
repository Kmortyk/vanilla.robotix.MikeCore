#!/usr/bin/env python

import sys

import roslib
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

roslib.load_manifest('mike_camera')

if __name__ == "__main__":
    # init node
    pub = rospy.Publisher('camera_topic', String, queue_size=10)
    bridge = CvBridge()

    rospy.init_node('mike_camera', anonymous=True)
    rate = rospy.Rate(10)  # hz
    # init camera
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        # capture frame-by-frame
        ret, frame = cap.read()
        rospy.loginfo("publish image")
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        rate.sleep()
