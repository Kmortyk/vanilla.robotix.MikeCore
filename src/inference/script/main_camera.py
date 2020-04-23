#!/usr/bin/env python3
from src.inference.script.model.ssd300MobileNet import SSD
from src.inference.script.ui.videotest import VideoTest
import os

# --- Config -----------------------------------------------------------------------------------------------------------

CLASS_NAMES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

WEIGHTS = 'weights' + os.sep + 'MobileNetSSD300weights_voc_2007_class20.hdf5'
INPUT_SHAPE = (300, 300, 3)
NUM_CLASSES = len(CLASS_NAMES)

# --- Model ------------------------------------------------------------------------------------------------------------

model = SSD(INPUT_SHAPE, num_classes=NUM_CLASSES)
model.load_weights('weights' + os.sep + WEIGHTS)

vid_test = VideoTest(CLASS_NAMES, model, INPUT_SHAPE)
vid_test.run(0)
