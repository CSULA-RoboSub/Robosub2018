import cv2
import numpy as np


class DicePreprocessor:

    def __init__(self):
        self.lower = np.array([0, 0, 0], 'uint8') # olins orig - works for dots
        self.upper = np.array([255, 255, 93], 'uint8') # olins orig - works for dots
        self.dots_lower = np.array([0, 0, 0], 'uint8') # any lighting
        self.dots_upper = np.array([180, 255, 80], 'uint8') # any lighting
        self.detect_dots = False
        self.min_cont_size = 100
        self.max_cont_size = 1000
        self.roi_size = 300
        self.ratio_lower = 0.85
        self.ratio_upper = 1.15
        self.kernel = np.ones((5, 5), np.uint8)

        
    def preprocess(self, img):
        if (self.detect_dots):
            mask = cv2.inRange(img, self.dots_lower, self.dots_upper)
        else:
            mask = cv2.inRange(img, self.lower, self.upper)
        output = cv2.bitwise_and(img, img, mask=mask)
        return output, mask

    def set_lower_color(self, task_name, lower):
        self.lower = np.array(lower, 'uint8')
        print 'lower is set to {} for {}'.format(lower, task_name)
        
    def set_upper_color(self, task_name, upper):
        self.upper = np.array(upper, 'uint8')
        print 'upper is set to {} for {}'.format(upper, task_name)

    def filter_contours(self, frame_contours):
        new_cont_list = []
        for cont in frame_contours:
            cont_len = len(cont)
            if self.min_cont_size < cont_len < self.max_cont_size:
                new_cont_list.append(cont)
        filtered_contours = np.array(new_cont_list)
        return filtered_contours
    

    def get_interest_regions(self, frame):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_filter_frame, mask = self.preprocess(hsv_frame)

        close_frame = cv2.morphologyEx(color_filter_frame, cv2.MORPH_CLOSE, self.kernel)
        erode_frame = cv2.erode(close_frame, self.kernel, iterations=1)
        dilate_frame = cv2.dilate(erode_frame, self.kernel, iterations=3)

        hsv2bgr_frame = cv2.cvtColor(dilate_frame, cv2.COLOR_HSV2BGR) # change color space to BGR
        grayscale_frame = cv2.cvtColor(hsv2bgr_frame, cv2.COLOR_BGR2GRAY) # to grayscale

        thresh_frame = cv2.adaptiveThreshold(grayscale_frame, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 3)
        frame_c, frame_contours, frame_hierarchy = cv2.findContours(thresh_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        frame_filtered_contours = self.filter_contours(frame_contours)
        
        boxes = [cv2.boundingRect(c) for c in frame_contours]

        # interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size and (self.ratio_lower < (float(b[2])/float(b[3])) < self.ratio_upper)]
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]

        return interest_regions
