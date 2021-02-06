import imutils
import cv2


class ResizePreprocessor:
    def __init__(self, width, height, inter=cv2.INTER_AREA):
        self.width = width
        self.height = height
        self.inter = inter

    def preprocess(self, image):
        (h, w) = image.shape[:2]
        # deltas to crop bigger dimension
        dw = dh = 0

        if w < h:
            image = imutils.resize(image, width=self.width, inter=self.inter)
            dh = int((image.shape[0] - self.height) / 2)
        else:
            image = imutils.resize(image, height=self.height, inter=self.inter)
            dw = int((image.shape[1] - self.width) / 2)

        shape = image.shape[1]

        # perform the crop
        (h, w) = image.shape[:2]
        image = image[dh:h-dh, dw:w-dw]

        # resize last time to ensure a fixed size
        return cv2.resize(image, (self.width, self.height), interpolation=self.inter), shape

