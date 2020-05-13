#!/usr/bin/env python3
# std
import sys
sys.path.append("./")

import numpy as np
import cv2
import os

# ros
import rospy
import inference.msg._Bboxes as Bboxes
import inference.msg._Bbox as Bbox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# inference
from src.inference.script.tf.model import TFModel
from src.inference.script.model.ssd300MobileNet import SSD
from src.inference.script.preprocess.maxsizeproc import MaxSizePreprocessor
from src.inference.script.preprocess.simpleproc import SimplePreprocessor
from src.inference.script.preprocess.resizeproc import ResizePreprocessor
from src.inference.script.preprocess.arrproc import ImageToArrayPreprocessor
from src.inference.script.preprocess.fitresizeproc import FitSizePreprocessor
from src.inference.script.ssd.utils import BBoxUtility

# --- Config -----------------------------------------------------------------------------------------------------------

obj_publisher = None

CLASSES = {0: "background", 1: "bottle", 2: "soup", 3: "flashlight", 4: "knife"}
FROZEN_GRAPH = "/home/jetson/models/frozen_inference_graph.pb"
CAM_INDEX = 1

def draw_rect(dst, x_min=0, x_max=0, y_min=0, y_max=0, conf=0.5, label="object", normalized=False, color=(172, 217, 153)):
    # scale the bounding box from the range [0,1] to [0, width], [0, height]
    if normalized:
        height, width = dst.shape[:2]
        x_min *= width
        y_min *= height
        x_max *= width
        y_max *= height
    # convert float to int
    x_min, y_min = int(x_min), int(y_min)
    x_max, y_max = int(x_max), int(y_max)
    # draw the prediction on the output image
    label = "{}: {:.2f}".format(label, conf)
    cv2.rectangle(dst, (x_min, y_min), (x_max, y_max), color, 2)
    y = y_min - 10 if y_min - 10 > 10 else y_min + 10
    cv2.putText(dst, label, (x_min, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)


proc = MaxSizePreprocessor(300)
model = TFModel(FROZEN_GRAPH)

def predict(msg):
    image = CvBridge().imgmsg_to_cv2(msg)
    image = proc.preprocess(image)
    output = image.copy()

    boxes, scores, labels = model.predict(image)
    for box, score, label_id in zip(boxes, scores, labels):
        label = CLASSES[label_id]
        print(f"label_id={label_id} label={label} box={box}")



if __name__ == '__main__':
    rospy.init_node('mike_inference', anonymous=True)
    obj_publisher = rospy.Publisher('/bboxes', Bboxes.Bboxes, queue_size=10)
    rospy.Subscriber('mike_camera/raw', Image, predict)
    rospy.spin()

