#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():
    print(sys.version)
    rospy.init_node('mike_inference', anonymous=True)
    # rospy.Subscriber('mike_camera/raw', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
