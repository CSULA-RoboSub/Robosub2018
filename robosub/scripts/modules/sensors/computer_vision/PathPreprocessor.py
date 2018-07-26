import utils
import cv2
import numpy as np

class PathPreprocessor():

    def __init__(self):
        self.lower_thresh = np.array([0, 150, 0], 'uint8')
        self.upper_thresh = np.array([20, 255, 255], 'uint8')
        self.roi_size = 1000
        self.min_cont_size = 100 # min contours size
        self.max_cont_size = 2000 # max contours size

    def filter_contours(self, frame_contours):
        new_cont_list = []
        for cont in frame_contours:
            cont_len = len(cont)
            if ( (cont_len > self.min_cont_size) and (cont_len < self.max_cont_size) ):
                new_cont_list.append(cont)
        filtered_contours = np.array(new_cont_list)
        return filtered_contours

    def set_lower_color(self, task_name, lower):
        self.lower = np.array(lower, 'uint8')
        print 'lower is set to {} for {}'.format(lower, task_name)
        
    def set_upper_color(self, task_name, upper):
        self.upper = np.array(upper, 'uint8')
        print 'upper is set to {} for {}'.format(upper, task_name)

    def preprocess(self, img):
        mask = cv2.inRange(img, self.lower_thresh, self.upper_thresh)
        output = cv2.bitwise_and(img, img, mask = mask)
        return output, mask

    def get_interest_regions(self, frame):
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img_color_filt, mask = self.preprocess(img_hsv)
        im, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # filtered_contours = self.filter_contours(contours)
        # boxes = [cv2.boundingRect(c) for c in filtered_contours]
        boxes = [cv2.boundingRect(c) for c in contours]
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]

        return interest_regions, contours