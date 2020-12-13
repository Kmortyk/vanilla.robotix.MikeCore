import sys
import threading
import time

sys.path.append("./")

from src.inference.script.trt.model import TrtModel
import src.inference.script.trt.config as config
import cv2

LABELS = ["background", "bottle", "soup"]
cap = cv2.VideoCapture(0)
model = TrtModel(model=config.model_ssd_mobilenet_v2_coco_2018_03_29,
                labels=LABELS)


def callback():
    start_time = time.time()
    secs = 5  # displays the frame rate every 1 second
    counter = 0

    while True:
        # read frame from the camera
        ret, image = cap.read()
        output = model.predict_draw(image)
        # show the output image
        cv2.imshow("Output", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        counter += 1
        if (time.time() - start_time) > secs:
            fps = (counter / (time.time() - start_time))
            #print("FPS: " + str(fps))
            counter = 0
            start_time = time.time()


if __name__ == "__main__":
    worker_thread1 = threading.Thread(target=callback)
    worker_thread1.start()
    worker_thread1.join()
