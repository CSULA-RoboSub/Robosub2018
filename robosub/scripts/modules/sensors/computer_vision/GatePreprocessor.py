import utils
import cv2
import numpy as np


class GatePreprocessor:

    def __init__(self):
        self.lower = np.array([0, 100, 100], 'uint8') # lower color value
        self.upper = np.array([60, 255, 255], 'uint8') # upper color value
        self.min_cont_size = 100 # min contours size
        self.max_cont_size = 2000 # max contours size
        self.roi_size = 400 # box size
        self.morph_ops = False # testing
        self.kernel = np.ones( (5, 5), np.uint8) # basic filter


    # color filtering
    def preprocess(self,  img):
        mask = cv2.inRange(img, self.lower, self.upper)
        output = cv2.bitwise_and(img, img, mask=mask)
        return output, mask

    def color_subtract(self, frame):
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

        blur = cv2.bilateralFilter(frame, 9, 100, 100) # blur

        frame_hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) # to HSV colorspace
        
        pimage, mask = self.preprocess(frame_hsv)
        imgray = cv2.cvtColor(pimage, cv2.COLOR_BGR2GRAY)
        
        #imgray = self.color_subtract(frame) # new test method - instead of color filter preproces
        
        flag, binary_image = cv2.threshold(imgray, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        if(self.morph_ops):
            erode_frame = cv2.erode(binary_image, self.kernel, iterations=1) # fade/trim
            open_frame = cv2.morphologyEx(erode_frame, cv2.MORPH_OPEN, self.kernel) # remove specs
            close_frame = cv2.morphologyEx(open_frame, cv2.MORPH_CLOSE, self.kernel) # fill in
            dilate_frame = cv2.dilate(close_frame, self.kernel, iterations=1) # make chubby

        #im, contours, ret = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        im, contours, ret = cv2.findContours(dilate_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # filter the contours based on size
        new_contours_list = []
        for cont in contours:
            if ( (len(cont) > self.min_cont_size) and (len(cont) < self.max_cont_size) ):
                new_contours_list.append(cont)
        filtered_contours = np.array(new_contours_list)

        boxes = [cv2.boundingRect(c) for c in filtered_contours]
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]
        
        return interest_regions
