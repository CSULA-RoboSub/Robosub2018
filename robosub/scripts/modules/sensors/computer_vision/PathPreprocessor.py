import utils
import cv2
import numpy as np

class PathPreprocessor():

    def __init__(self):
        self.lower_thresh = np.array([0, 150, 0], 'uint8')
        self.upper_thresh = np.array([10, 255, 255], 'uint8')
        self.roi_size = 300
    def preprocess(self, img):
        mask = cv2.inRange(img, self.lower_thresh, self.upper_thresh)
        output = cv2.bitwise_and(img, img, mask = mask)
        return output, mask

    def get_interest_regions(self, frame):
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img_color_filt, mask = color_filter(img_hsv, [lower_thresh, upper_thresh] )
        im, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        boxes = [cv2.boundingRect(c) for c in contours]
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]

        return interest_regions