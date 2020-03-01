#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
import cv_bridge
import cv2

def callback(msg):
    frame = CvBridge().imgmsg_to_cv2(msg)
    cv2.imshow("Image", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return
    rospy.loginfo("Get image")

def listener():
    rospy.init_node('mike_inference', anonymous=True)
    rospy.Subscriber('mike_camera/raw', Image, callback)
    
    rospy.spin()

if __name__ == '__main__':
    cv2.namedWindow('Image',cv2.WINDOW_AUTOSIZE)
    listener()
