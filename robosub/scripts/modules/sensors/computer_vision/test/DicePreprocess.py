import cv2
import numpy as np


class DicePreprocessor:

    def __init__(self):
        self.lower = [0, 80, 80]
        self.upper = [170, 255, 255]
        self.roi_size = 300

    def preprocess(self,  frame):

        imhsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        imhsv_blur = cv2.medianBlur(imhsv,11)
        im_blur = cv2.medianBlur(frame,25)

        hsv_lower = np.array([40,40,40])
        hsv_upper = np.array([100,255,255])

        hsv_mask = cv2.inRange(imhsv_blur,hsv_lower,hsv_upper)
        hsv_filter = cv2.bitwise_and(imhsv_blur,imhsv_blur,mask=hsv_mask)

        lower = np.array(self.lower, dtype='uint8')
        upper = np.array(self.upper, dtype='uint8')
        mask = cv2.inRange(frame,  lower,  upper)

        output = cv2.bitwise_and(frame,  frame,  mask=mask)

        return output, mask, imhsv, imhsv_blur, hsv_filter

    def get_interest_regions(self,frame):


        height, width, lines = frame.shape
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

    def show_preprocess_images(self,frame):
        height, width, lines = frame.shape
        center = (width / 2, height / 2)
        pimage,  mask, hsv, hsv_blur, hsv_filter = self.preprocess(frame)
        imgray = cv2.cvtColor(pimage, cv2.COLOR_BGR2GRAY)
        flag, binary_image = cv2.threshold(imgray, 85, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        edges = cv2.Canny(binary_image, 50, 150)

        im, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.imshow('pimage',pimage)
        cv2.imshow('hsv',hsv)
        cv2.imshow('hsv_blur',hsv_blur)
        cv2.imshow('hsv_filter',hsv_filter)
        cv2.imshow('mask',mask)

