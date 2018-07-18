import cv2
import numpy as np


class DicePreprocessor:

    def __init__(self):
        self.lower = np.array([0, 80, 80], 'uint8')
        self.upper = np.array([170, 255, 255], 'uint8')
        self.dots_lower = [0, 0, 0]
        self.dits_upper = [180, 255, 60]
        self.roi_size = 300
        self.detect_dots = False

    def preprocess(self, img):
        mask = cv2.inRange(img, self.lower, self.upper)
        output = cv2.bitwise_and(img, img, mask=mask)
        return output, mask

    def get_interest_regions(self,frame):
        
        height, width, ch = frame.shape
        center = (width / 2, height / 2)
        pimage, mask ,hsv,hsv_blur,hsv_filter = self.preprocess(frame)
        imgray = cv2.cvtColor(pimage, cv2.COLOR_BGR2GRAY)
        flag, binary_image = cv2.threshold(imgray, 85, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        edges = cv2.Canny(binary_image, 50, 150)

        im, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.imshow('pimage',pimage)
        #cv2.imshow('mask',mask)
        boxes = [cv2.boundingRect(c) for c in contours]

        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]

        return interest_regions
