#!/usr/bin/env python3
import sys
sys.path.append("./")

import cv2
from src.inference.script.preprocess import FitSizePreprocessor
from src.inference.script.trt.model_ros import TrtModel
import src.inference.script.trt.config as config
import rospy
import inference.msg._Bboxes as Bboxes
import inference.msg._Bbox as Bbox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

ROS_NODE_RATE_HZ = 5
IMAGE_SIZE = 300
LABELS = ["", "bottle"]
SHOW_IMAGE = True
CAM_INDEX = 1

if __name__ == "__main__":
    # open camera with VideoCapture
    cap = cv2.VideoCapture(CAM_INDEX)
    # initialize preprocessor
    prep = FitSizePreprocessor(IMAGE_SIZE)
    # create model if needed (trt optimizations)
    model = TrtModel(model=config.model_ssd_mobilenet_v2_coco_2018_03_29, labels=LABELS)
    # image copy if showing enabled
    copy = None

    # init ROS node
    rospy.init_node('mike_inference', anonymous=True)
    # create bbox publisher
    obj_publisher = rospy.Publisher('/bboxes', Bboxes.Bboxes, queue_size=10)
    # initialize ROS node working rate
    rate = rospy.Rate(ROS_NODE_RATE_HZ)

    rospy.loginfo("[INFO] start trt predicting node...")
    while not rospy.is_shutdown():
        # capture image from the camera
        ret, image = cap.read()

        # if showing enabled - copy preprocessed image
        if SHOW_IMAGE:
            copy = prep.preprocess(image.copy())

        # predict and publish predicted objects
        objs = model.predict_bboxes(image)
        obj_publisher.publish(objs)

        # show image with bounding boxes if needed
        if SHOW_IMAGE:
            for bbox in objs.bboxes:
                cv2.rectangle(copy, (bbox.x_min, bbox.y_min), (bbox.x_max, bbox.y_max), (172, 217, 153), 2)
                cv2.putText(copy, bbox.label, (bbox.x_min + 10, bbox.y_min + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("Output", copy)
            # no wait user for output
            if cv2.waitKey(1) & 0xFF == ord('q'):
                sys.exit(0)

        # sleep with given rate
        rate.sleep()
