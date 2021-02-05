#!/usr/bin/env python3
import sys
sys.path.append("./")

from src.inference.script.preprocess import *

import cv2
from src.inference.script.trt.model_ros_nms import TrtModel
import src.inference.script.trt.config as config
import rospy

import inference.msg._Bboxes as Bboxes
import inference.msg._Bbox as Bbox

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SHOW_FRAME_SIZE_X = 1198
SHOW_FRAME_SIZE_Y = 900
NEURAL_FRAME_SIZE = 300

NEURAL_TO_SHOW_COEFFICIENT_X = SHOW_FRAME_SIZE_X / NEURAL_FRAME_SIZE
NEURAL_TO_SHOW_COEFFICIENT_Y = SHOW_FRAME_SIZE_Y / NEURAL_FRAME_SIZE

def gstreamer_pipeline(
    capture_width=3280,
    capture_height=2464,
    display_width=SHOW_FRAME_SIZE_X,
    display_height=SHOW_FRAME_SIZE_Y,
    framerate=21,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


LABELS = ["background", "bottle", "soup"]
SHOW_IMAGE = True
model = TrtModel(model=config.model_ssd_inception_v2_coco_2017_11_17, labels=LABELS)
obj_publisher = None
# prep = ResizePreprocessor(SHOW_FRAME_SIZE, SHOW_FRAME_SIZE)
# copy = None
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)


def convert_neural_to_show(objs):
    try:
        for bbox in objs.bboxes:
            bbox.x_min *= NEURAL_TO_SHOW_COEFFICIENT_X
            bbox.y_min *= NEURAL_TO_SHOW_COEFFICIENT_Y
            bbox.x_max *= NEURAL_TO_SHOW_COEFFICIENT_X
            bbox.y_max *= NEURAL_TO_SHOW_COEFFICIENT_Y
    except AttributeError:
        rospy.logerr("AttributeError: 'list' object has no attribute 'bboxes'")
    return objs


def step():
    #print(gstreamer_pipeline())
    if not cap.isOpened():
        rospy.logerr("Camera doesn't open")
        return
    # read frame from the camera
    ret, image = cap.read()
    rospy.loginfo("[INFO] receive image from the mike_camera/raw")

    # if show image - create copy
    # if SHOW_IMAGE:
    #     copy = prep.preprocess(image.copy())

    # predict and publish predicted objects
    objs = model.predict_bboxes(image)
    objs = convert_neural_to_show(objs)
    obj_publisher.publish(objs)

    # show image with bounding boxes if needed
    if SHOW_IMAGE:
        for bbox in objs.bboxes:
            cv2.rectangle(image, (bbox.x_min, bbox.y_min), (bbox.x_max, bbox.y_max), (172, 217, 153), 2)
            cv2.putText(image, bbox.label, (bbox.x_min + 10, bbox.y_min + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Output", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return


if __name__ == '__main__':
    rospy.init_node('mike_inference', anonymous=True)
    obj_publisher = rospy.Publisher('/bboxes', Bboxes.Bboxes, queue_size=10)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("[INFO] camera step...")
        step()
        r.sleep()
