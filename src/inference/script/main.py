#!/usr/bin/env python3
# std
import numpy as np
import cv2
import os

# ros
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# inference
from keras.applications.imagenet_utils import preprocess_input
from src.inference.script.model.ssd300MobileNet import SSD
from src.inference.script.preprocess.maxsizeproc import MaxSizePreprocessor
from src.inference.script.ssd.utils import BBoxUtility

# --- Config -----------------------------------------------------------------------------------------------------------

CLASS_NAMES = [
    'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow',
    'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor'
]

WEIGHTS = 'weights' + os.sep + 'MobileNetSSD300weights_voc_2007_class20.hdf5'
INPUT_SHAPE = (300, 300, 3)
NUM_CLASSES = len(CLASS_NAMES)

# --- Model ------------------------------------------------------------------------------------------------------------

model = SSD(INPUT_SHAPE, num_classes=NUM_CLASSES)
model.load_weights('weights' + os.sep + WEIGHTS)
bbox_util = BBoxUtility(NUM_CLASSES)

proc = [
    MaxSizePreprocessor(1000)
]


def predict(msg):
    rospy.loginfo("get image")
    image = CvBridge().imgmsg_to_cv2(msg)
    image = image.img_to_array(image)

    rospy.loginfo("predicting...")
    # preprocess inputs
    inputs = [image]
    inputs = preprocess_input(np.array(inputs))
    # predict inputs
    preds = model.predict(inputs, batch_size=1, verbose=1)
    result = bbox_util.detection_out(preds)[0]

    # parse the outputs.
    det_label = result[:, 0]
    det_conf = result[:, 1]
    det_x_min = result[:, 2]
    det_y_min = result[:, 3]
    det_x_max = result[:, 4]
    det_y_max = result[:, 5]

    # Get detections with confidence higher than 0.6.
    top_indices = [i for i, conf in enumerate(det_conf) if conf >= 0.6]

    top_conf = det_conf[top_indices]
    top_label_indices = det_label[top_indices].tolist()
    top_x_min = det_x_min[top_indices]
    top_y_min = det_y_min[top_indices]
    top_x_max = det_x_max[top_indices]
    top_y_max = det_y_max[top_indices]

    for i in range(top_conf.shape[0]):
        x_min = int(round(top_x_min[i] * image.shape[1]))
        y_min = int(round(top_y_min[i] * image.shape[0]))
        x_max = int(round(top_x_max[i] * image.shape[1]))
        y_max = int(round(top_y_max[i] * image.shape[0]))
        score = top_conf[i]
        label = CLASS_NAMES[int(top_label_indices[i])]


if __name__ == '__main__':
    cv2.namedWindow('Image', cv2.WINDOW_AUTOSIZE)
    rospy.init_node('mike_inference', anonymous=True)
    rospy.Subscriber('mike_camera/raw', Image, predict)
    rospy.spin()
