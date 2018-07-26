import utils
import cv2
import numpy as np


class GatePreprocessor:

    def __init__(self):
        # self.lower = np.array([0, 100, 0], 'uint8') # lower color value
        # self.upper = np.array([180, 200, 150], 'uint8') # upper color value
        #bright values
        # self.lower = np.array([0, 99, 0], 'uint8') # lower color value    
        # self.upper = np.array([180, 200, 155], 'uint8') # upper color value
        #bright values 2
        # self.lower = np.array([0, 89, 0], 'uint8') # lower color value    
        # self.upper = np.array([180, 254, 80], 'uint8') # upper color value
        #bright values 3
        # self.lower = np.array([89, 89, 39], 'uint8') # lower color value    
        # self.upper = np.array([149, 200, 80], 'uint8') # upper color value
        #dark values--------------------------------
        # self.lower = np.array([0, 69, 0], 'uint8') # lower color value  
        # self.upper = np.array([180, 254, 80], 'uint8') # upper color value
        #dark val 2
        # self.lower = np.array([1, 69, 89], 'uint8') # lower color value  
        # self.upper = np.array([180, 254, 99], 'uint8') # upper color value
        #dark val 3
        self.lower = np.array([109, 71, 88], 'uint8') # lower color value  
        self.upper = np.array([139, 169, 99], 'uint8') # upper color value
        self.min_cont_size = 100 # min contours size      
        self.max_cont_size = 2000 # max contours size
        self.roi_size = 400 # box size
        self.morph_ops = True # testing
        self.kernel = np.ones( (5, 5), np.uint8) # basic filter


    # color filtering
    def preprocess(self, img):
        mask = cv2.inRange(img, self.lower, self.upper)
        output = cv2.bitwise_and(img, img, mask=mask)
        return output, mask


    # expects a
    def set_lower_color(self, task_name, lower):
        self.lower = np.array(lower, 'uint8')
        print 'lower is set to {} for {}'.format(lower, task_name)

        
    def set_upper_color(self, task_name, upper):
        self.upper = np.array(upper, 'uint8')
        print 'upper is set to {} for {}'.format(upper, task_name)

    
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

    def filter_contours(self, frame_contours):
        new_cont_list = []
        for cont in frame_contours:
            cont_len = len(cont)
            if ( (cont_len > self.min_cont_size) and (cont_len < self.max_cont_size) ):
                new_cont_list.append(cont)
        filtered_contours = np.array(new_cont_list)
        return filtered_contours


    # returns ROI
    def get_interest_regions(self, frame):
        
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # to HSV colorspace
        
        color_filt_frame, mask = self.preprocess(hsv_frame) # color filtering
        
        close_frame = cv2.morphologyEx(color_filt_frame, cv2.MORPH_CLOSE, self.kernel) # fill in
        dilate_frame = cv2.dilate(close_frame, self.kernel, iterations=3) # make chubby

        hsv2bgr_frame = cv2.cvtColor(dilate_frame, cv2.COLOR_HSV2BGR) # change color space to BGR
        grayscale_frame = cv2.cvtColor(hsv2bgr_frame, cv2.COLOR_BGR2GRAY) # to grayscale

        ret, thresh_frame = cv2.threshold(grayscale_frame, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        frame_c, frame_contours, frame_heirarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours = self.filter_contours(frame_contours) # filter the contours based on size

        boxes = [cv2.boundingRect(c) for c in filtered_contours] # make boxes around contours
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]
        
        return interest_regions
