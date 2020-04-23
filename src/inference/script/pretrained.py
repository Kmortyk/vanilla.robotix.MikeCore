#!/usr/bin/env python3

import rospy
# cv send image shit
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# inference
from config import config
from preprocess.maxsizeproc import MaxSizePreprocessor
from predict.classes import get_class_name
from predict.tf_model import TFModel
from predict.ui import draw_bbox
# std lib
import cv2

model = VGG16(weights='imagenet', include_top=False)
proc = MaxSizePreprocessor(224)

def predict(msg):
    rospy.loginfo("get image")
    image = CvBridge().imgmsg_to_cv2(msg)

    rospy.loginfo("predicting...")
    # loop over the bounding box predictions
    boxes, scores, labels = model.predict(image)
    for box, score, label_id in zip(boxes, scores, labels):
        label = get_class_name(label_id, config.CLASSES)
        draw_bbox(image, box, score, label)

    # show the output image
    image = proc.process(image)
    x = image.img_to_array(img)
    x = np.expand_dims(x, axis=0)
    x = preprocess_input(x)

    features = model.predict(x)
    print(features)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit(0)


if __name__ == '__main__':
    cv2.namedWindow('Image', cv2.WINDOW_AUTOSIZE)
    rospy.init_node('mike_inference', anonymous=True)
    rospy.Subscriber('mike_camera/raw', Image, predict)
    rospy.spin()
