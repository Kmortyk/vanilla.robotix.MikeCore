import sys
import threading
import time

sys.path.append("./")

from trt.model import TrtModel
import trt.config as config
import cv2

LABELS = ["", "bottle", "soup"]
im_path = "/mnt/sda1/backup/Projects/Python/Rozenbroh/datasets/mike/images/mix2/mike-video-15-03-2020-14-38-42_90.5.jpg"
model = TrtModel(model=config.model_ssd_inception_v2_coco_2017_11_17, labels=LABELS)


def callback():
    # read frame from the camera
    im = cv2.imread(im_path)
    output = model.predict_draw(im)
    # show the output image
    cv2.imshow("Output", output)
    if cv2.waitKey(0):
        return


if __name__ == "__main__":
    worker_thread1 = threading.Thread(target=callback)
    worker_thread1.start()
    worker_thread1.join()
