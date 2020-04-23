import imutils


class MaxSizePreprocessor:
    def __init__(self, size):
        self.size = size

    def preprocess(self, image):
        height, width = image.shape[:2]
        # check to see if we should resize along the width
        if width > height and width > self.size:
            image = imutils.resize(image, width=self.size)
        # otherwise, check to see if we should resize along the height
        if height > width and height > self.size:
            image = imutils.resize(image, height=self.size)
        return image
