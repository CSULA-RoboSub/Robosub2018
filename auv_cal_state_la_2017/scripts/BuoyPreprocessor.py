import utils
import cv2
import numpy as np


class BuoyPreprocessor:

    def __init__(self):
        self.lower = [0, 60, 60]
        self.upper = [60, 255, 255]
        self.roi_size = 400

    def preprocess(self,  img):

        lower = np.array(self.lower, dtype='uint8')
        upper = np.array(self.upper, dtype='uint8')
        mask = cv2.inRange(img,  lower,  upper)
        output = cv2.bitwise_and(img,  img,  mask=mask)
        return output, mask

    def get_interest_regions(self,frame):

        height, width, lines = frame.shape
        center = (width / 2, height / 2)
        pimage, mask = self.preprocess(frame)
        imgray = cv2.cvtColor(pimage, cv2.COLOR_BGR2GRAY)
        flag, binary_image = cv2.threshold(imgray, 85, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        edges = cv2.Canny(binary_image, 50, 150)

        im, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.imshow('pimage',pimage)
        #cv2.imshow('mask',mask)
        boxes = [cv2.boundingRect(c) for c in contours]

        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]

        return interest_regions

