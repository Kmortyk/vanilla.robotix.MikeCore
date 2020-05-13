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
from src.inference.script.preprocess.resizeproc import ResizePreprocessor
from src.inference.script.preprocess.arrproc import ImageToArrayPreprocessor
from src.inference.script.preprocess.fitresizeproc import FitSizePreprocessor
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
    ResizePreprocessor(300, 300),
    #SimplePreprocessor(300, 300),
    #FitSizePreprocessor(300),
    ImageToArrayPreprocessor(),
]


from datetime import datetime

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

def predict(msg):
    image = CvBridge().imgmsg_to_cv2(msg)
    copy = proc[0].preprocess(image.copy())
    for p in proc:
        image = p.preprocess(image)
    # preprocess inputs
    inputs = [image]
    inputs = preprocess_input(np.array(inputs))
    # predict inputs
    with conf.session.as_default():
        with conf.session.graph.as_default():
            preds = model.predict(inputs, batch_size=1, verbose=0)
            result = bbox_util.detection_out(preds)[0]

    if len(result) == 0:
        return

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
        draw_rect(copy, conf=score, label=label, x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max)
        print(f"[INFO] publish predictions: {label}: ({x_min}, {y_min}, {x_max}, {y_max})")

    # cv2.imshow("Frame", copy)
    now = datetime.now()
    cv2.imwrite("/home/jetson/frames/image-" + now.strftime("%m-%d-%Y-%H-%M-%S") + ".jpg", copy)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #    exit(0)


if __name__ == '__main__':
    rospy.init_node('mike_inference', anonymous=True)
    obj_publisher = rospy.Publisher('/bboxes', Bboxes.Bboxes, queue_size=10)
    rospy.Subscriber('mike_camera/raw', Image, predict)
    rospy.spin()

