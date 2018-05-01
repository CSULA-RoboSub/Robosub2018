import utils
import cv2
import numpy as np


class GatePreprocessor:

    def __init__(self):
        self.lower = np.array([0, 50, 50], 'uint8') # lower color value
        self.upper = np.array([130, 250, 255], 'uint8') # upper color value
        self.min_cont_size = 20 # min contours size
        self.max_cont_size = 2000 # max contours size
        self.roi_size = 400 # box size


    # color filtering
    def preprocess(self,  img):
        mask = cv2.inRange(img, self.lower, self.upper)
        output = cv2.bitwise_and(img, img, mask=mask)
        return output, mask

    def color_subtract(frame):
        blue = frame.copy()
        green = frame.copy()
        red = frame.copy()
        
        blue[:, :, 0] = 255
        green[:, :, 1] = 255
        red[:, :, 2] = 255
        
        blue_gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
        green_gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
        red_gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)

        green_blue = green_gray - blue_gray

        return green_blue


    # returns ROI
    def get_interest_regions(self, frame):
        height, width, lines = frame.shape
        
        #pimage, mask = self.preprocess(frame)
        #imgray = cv2.cvtColor(pimage, cv2.COLOR_BGR2GRAY)
        
        imgray = color_subtract(frame) # new test method
        
        flag, binary_image = cv2.threshold(imgray, 127, 255, cv2.THRESH_TOZERO)
        im, contours, ret = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # filter the contours based on size
        new_contours_list = []
        for cont in contours:
            if ( (len(cont) > self.min_cont_size) and (len(cont) < self.max_cont_size) ):
                new_contours_list.append(cont)
        filtered_contours = np.array(new_contours_list)

        boxes = [cv2.boundingRect(c) for c in filtered_contours]
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]
        
        return interest_regions
