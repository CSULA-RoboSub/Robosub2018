import utils
import cv2
import numpy as np

class PlatformPreprocessor:

    def __init__(self):
        self.lower = np.array([90, 180, 90], 'uint8')
        self.upper = np.array([110, 255, 110], 'uint8')

    def preprocess(self, img):
        mask = cv2.inRange(img, self.lower, self.upper)
        output = cv2.bitwise_and(img, img, mask=mask)
        return output, mask

