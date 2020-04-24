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
from keras.applications.imagenet_utils import preprocess_input
from src.inference.script.model.ssd300MobileNet import SSD
from src.inference.script.preprocess.maxsizeproc import MaxSizePreprocessor
from src.inference.script.preprocess.simpleproc import SimplePreprocessor
from src.inference.script.preprocess.arrproc import ImageToArrayPreprocessor
from src.inference.script.ssd.utils import BBoxUtility

# config
from src.inference.script.config.sessionconfig import SessionConfig

# --- Config -----------------------------------------------------------------------------------------------------------

obj_publisher = None

CLASS_NAMES = [
    'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow',
    'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor'
]

BASE_PATH = '/home/jetson/vanilla.robotix.MikeCore/src/inference/'
WEIGHTS = BASE_PATH + os.sep + 'weights' + os.sep + 'MobileNetSSD300weights_voc_2007_class20.hdf5'
INPUT_SHAPE = (300, 300, 3)
NUM_CLASSES = len(CLASS_NAMES)

conf = SessionConfig()
conf.configure()

# --- Model ------------------------------------------------------------------------------------------------------------

model = SSD(INPUT_SHAPE, num_classes=NUM_CLASSES)
model.load_weights(WEIGHTS)
model._make_predict_function()
bbox_util = BBoxUtility(NUM_CLASSES)

proc = [
    SimplePreprocessor(300, 300),
    ImageToArrayPreprocessor(),
]


def predict(msg):
    image = CvBridge().imgmsg_to_cv2(msg)
    for p in proc:
        image = p.preprocess(image)

    # preprocess inputs
    inputs = [image]
    inputs = preprocess_input(np.array(inputs))
    
    # predict inputs
    with conf.session.as_default():
        with conf.session.graph.as_default():
            preds = model.predict(inputs, batch_size=1)
            result = bbox_util.detection_out(preds)[0]

    if len(result) == 0:
        rospy.loginfo("nothing was found")
        return

    # parse the outputs.
    det_label = result[:, 0]
    det_conf = result[:, 1]
    det_x_min = result[:, 2]
    det_y_min = result[:, 3]
    det_x_max = result[:, 4]
    det_y_max = result[:, 5]

    # get detections with confidence higher than 0.6.
    top_indices = [i for i, conf in enumerate(det_conf) if conf >= 0.6]

    top_conf = det_conf[top_indices]
    top_label_indices = det_label[top_indices].tolist()
    top_x_min = det_x_min[top_indices]
    top_y_min = det_y_min[top_indices]
    top_x_max = det_x_max[top_indices]
    top_y_max = det_y_max[top_indices]

    objs = Bboxes.Bboxes()
    for i in range(top_conf.shape[0]):
        obj = Bbox.Bbox()
        obj.x_min = int(round(top_x_min[i] * image.shape[1]))
        obj.y_min = int(round(top_y_min[i] * image.shape[0]))
        obj.x_max = int(round(top_x_max[i] * image.shape[1]))
        obj.y_max = int(round(top_y_max[i] * image.shape[0]))
        obj.score = top_conf[i]
        obj.label = CLASS_NAMES[int(top_label_indices[i])]
        objs.bboxes.append(obj)
        rospy.loginfo(f"[INFO] publish predictions: {obj.label}[{obj.score}]: ({obj.x_min}, {obj.y_min}, {obj.x_max}, {obj.y_max})")
    obj_publisher.publish(objs)

if __name__ == '__main__':
    rospy.init_node('mike_inference', anonymous=True)
    obj_publisher = rospy.Publisher('/bboxes', Bboxes.Bboxes, queue_size=10)
    rospy.Subscriber('mike_camera/raw', Image, predict)
    rospy.spin()
