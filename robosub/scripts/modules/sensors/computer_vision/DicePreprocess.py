import cv2
import numpy as np


class DicePreprocessor:

    def __init__(self):
        self.lower = np.array([0, 0, 0], 'uint8') # olins orig - works for dots
        self.upper = np.array([255, 255, 93], 'uint8') # olins orig - works for dots
        self.dots_lower = np.array([0, 0, 0], 'uint8') # any lighting
        self.dots_upper = np.array([180, 255, 80], 'uint8') # any lighting
        self.min_cont_size = 100
        self.max_cont_size = 1000
        self.roi_size = 300
        self.detect_dots = False

        self.ratio_lower = 0.85
        self.ratio_upper = 1.15
    def preprocess(self, img):
        if (self.detect_dots):
            mask = cv2.inRange(img, self.dots_lower, self.dots_upper)
        else:
            mask = cv2.inRange(img, self.lower, self.upper)
        output = cv2.bitwise_and(img, img, mask=mask)
        return output, mask


    def filter_contours(self, frame_countours):
        new_cont_list = []
        for cont in frame_contours:
            cont_len = len(cont)
            if self.min_cont_size < cont_len < self.max_cont_size:
                new_cont_list.append(cont)
        filtered_contours = np.array(new_cont_list)
        return filtered_contours
    

    def get_interest_regions(self, frame):

        pimage, mask = self.preprocess(frame)
        imgray = cv2.cvtColor(pimage, cv2.COLOR_BGR2GRAY)
        flag, binary_image = cv2.threshold(imgray, 85, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # edges = cv2.Canny(binary_image, 50, 150)

        # im, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        im, contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        boxes = [cv2.boundingRect(c) for c in contours]

        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size and (self.ratio_lower < (float(b[2])/float(b[3])) < self.ratio_upper)]

        return interest_regions
