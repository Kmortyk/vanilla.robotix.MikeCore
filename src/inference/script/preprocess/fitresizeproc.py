import imutils
import numpy as np


class FitSizePreprocessor:
    def __init__(self, size):
        self.size = size

    def preprocess(self, image):
        height, width = image.shape[:2]
        canvas = np.zeros((self.size, self.size, 3), dtype="uint8")

        if width > height and width > self.size:
            image = imutils.resize(image, width=self.size)

        if height > width and height > self.size:
            image = imutils.resize(image, height=self.size)

        height, width = image.shape[:2]
        dy = int(abs(height - self.size) * 0.5)
        dx = int(abs(width - self.size) * 0.5)

        canvas[dy:height + dy, dx:width + dx] = image
        return canvas

