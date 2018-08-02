import utils
import cv2
import numpy as np
from utilities.filters import *

class GatePreprocessor:

    def __init__(self):
        # old values-----------------------------------------------------
        # self.lower = np.array([0, 100, 0], 'uint8') # lower color value
        # self.upper = np.array([179, 200, 150], 'uint8') # upper color value
        #bright values
        # self.lower = np.array([0, 99, 0], 'uint8') # lower color value    
        # self.upper = np.array([179, 200, 155], 'uint8') # upper color value
        #bright values 2
        # self.lower = np.array([0, 89, 0], 'uint8') # lower color value    
        # self.upper = np.array([179, 254, 80], 'uint8') # upper color value
        #bright values 3
        # self.lower = np.array([89, 89, 39], 'uint8') # lower color value    
        # self.upper = np.array([149, 200, 80], 'uint8') # upper color value
        #dark values--------------------------------
        # self.lower = np.array([0, 69, 0], 'uint8') # lower color value  
        # self.upper = np.array([179, 254, 80], 'uint8') # upper color value
        #dark val 2
        # self.lower = np.array([1, 69, 89], 'uint8') # lower color value  
        # self.upper = np.array([179, 254, 99], 'uint8') # upper color value
        #dark val 3 (best)
        # self.lower = np.array([109, 71, 88], 'uint8') # lower color value  
        # self.upper = np.array([139, 169, 99], 'uint8') # upper color value
        #----------------------------------------------------------------

        #current values--------------------------------------------------
        #orangeish red and blueish red vals 
        #bright 1
        # self.lower_red_orange = np.array([0, 87, 1], 'uint8')
        # self.upper_red_orange = np.array([31, 255, 254], 'uint8')

        # self.lower_red_blue = np.array([131, 87, 1], 'uint8')
        # self.upper_red_blue = np.array([180, 255, 254], 'uint8')

        #bright 2
        # self.lower_red_orange = np.array([0, 109, 2], 'uint8')
        # self.upper_red_orange = np.array([31, 255, 254], 'uint8')

        # self.lower_red_blue = np.array([131, 109, 2], 'uint8')
        # self.upper_red_blue = np.array([180, 255, 254], 'uint8')

        #bright 3
        # self.lower_red_orange = np.array([0, 99, 2])
        # self.upper_red_orange = np.array([31, 255, 254])

        # self.lower_red_blue = np.array([123, 99, 2])
        # self.upper_red_blue = np.array([180, 255, 254])

        #dark 1
        self.lower_red_orange = np.array([0, 16, 29], 'uint8')
        self.upper_red_orange = np.array([31, 255, 255], 'uint8')

        self.lower_red_blue = np.array([130, 16, 29], 'uint8')
        self.upper_red_blue = np.array([180, 255, 255], 'uint8')

        ''' BGR color values '''
        #self.lower_bgr = np.array([212, 0, 0], 'uint8')
        #self.upper_bgr = np.array([200, 255, 255], 'uint8')

        self.lower_bgr = np.array([130, 150, 210], 'uint8')
        self.upper_bgr = np.array([255, 255, 255], 'uint8')

        #----------------------------------------------------------------
        #flags only enable one
        self.use_bgr = True
        self.use_hsv_and_bgr = False
        self.use_bgr_and_hsv = False
        #-------------------------------------------------

        self.min_cont_size = 100 # min contours size
        self.max_cont_size = 1000 # max contours size
        self.roi_size = 1000 # box size
        self.morph_ops = True # testing

        ''' KERNEL '''
        #self.kernel = np.ones( (5, 5), np.uint8) # basic filter
        self.kernel = kernel_diag_pos
        # self.kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(5,5))
        # self.kernel = np.array([[0, 1, 1, 1, 0],
        #                         [0, 1, 1, 1, 0],
        #                         [1, 1, 1, 1, 1],
        #                         [0, 1, 1, 1, 0],
        #                         [0, 1, 1, 1, 0]])


    # color filtering
    def preprocess(self, img):
        # mask = cv2.inRange(img, self.lower, self.upper)
        # output = cv2.bitwise_and(img, img, mask=mask)

        #-----------------------------------
        #saved test code
        # mask_preblur = cv2.addWeighted(mask_red_orange, 1.0, mask_red_blue, 1.0, 0)
        # mask = cv2.GaussianBlur(mask_preblur,(9,9),0)
        #-----------------------------------

        if self.use_bgr:
            mask = cv2.inRange(img, self.lower_bgr, self.upper_bgr)
            output = cv2.bitwise_and(img, img, mask=mask)

        elif self.use_hsv_and_bgr:
            hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # to HSV colorspace

            mask_red_orange = cv2.inRange(hsv_frame, self.lower_red_orange, self.upper_red_orange)
            mask_red_blue = cv2.inRange(hsv_frame, self.lower_red_blue, self.upper_red_blue)
            mask_hsv = cv2.addWeighted(mask_red_orange, 1.0, mask_red_blue, 1.0, 0)

            color_filt_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask_hsv)

            close_frame = cv2.morphologyEx(color_filt_frame, cv2.MORPH_CLOSE, self.kernel) # fill in
            dilate_frame = cv2.dilate(close_frame, self.kernel, iterations=3) # make chubby
            output_hsv = cv2.cvtColor(dilate_frame, cv2.COLOR_HSV2BGR) # change color space to BGR

            mask = cv2.inRange(output_hsv, self.lower_bgr, self.upper_bgr)
            output = cv2.bitwise_and(output_hsv, output_hsv, mask=mask)

        elif self.use_bgr_and_hsv:
            mask_bgr = cv2.inRange(img, self.lower_bgr, self.upper_bgr)
            bgr_frame = cv2.bitwise_and(img, img, mask=mask_bgr)

            hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV) # to HSV colorspace

            mask_red_orange = cv2.inRange(hsv_frame, self.lower_red_orange, self.upper_red_orange)
            mask_red_blue = cv2.inRange(hsv_frame, self.lower_red_blue, self.upper_red_blue)
            mask = cv2.addWeighted(mask_red_orange, 1.0, mask_red_blue, 1.0, 0)

            color_filt_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)        

            close_frame = cv2.morphologyEx(color_filt_frame, cv2.MORPH_CLOSE, self.kernel) # fill in
            dilate_frame = cv2.dilate(close_frame, self.kernel, iterations=3) # make chubby
            output = cv2.cvtColor(dilate_frame, cv2.COLOR_HSV2BGR) # change color space to BGR
            
        else:
            hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # to HSV colorspace

            mask_red_orange = cv2.inRange(hsv_frame, self.lower_red_orange, self.upper_red_orange)
            mask_red_blue = cv2.inRange(hsv_frame, self.lower_red_blue, self.upper_red_blue)
            mask = cv2.addWeighted(mask_red_orange, 1.0, mask_red_blue, 1.0, 0)
            
            color_filt_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)        

            close_frame = cv2.morphologyEx(color_filt_frame, cv2.MORPH_CLOSE, self.kernel) # fill in
            dilate_frame = cv2.dilate(close_frame, self.kernel, iterations=3) # make chubby
            output = cv2.cvtColor(dilate_frame, cv2.COLOR_HSV2BGR) # change color space to BGR

        # mask_inv = cv2.bitwise_not(mask)
        # output = cv2.bitwise_and(img, img, mask=mask_inv)

        return output, mask

    
    def get_lower_color(self):
        return self.lower.tolist()

    
    # expects a list - gets converted to a numpy array in setter
    def set_lower_color(self, task_name, lower):
        self.lower = np.array(lower, 'uint8')
        print 'lower is set to {} for {}'.format(lower, task_name)

    # returns a list - gets converted from a numpy array
    def get_upper_color(self):
        return self.upper.tolist()

    
    # expects a list - gets converted to a numpy array in setter
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
    # def get_interest_regions(self, frame):
    
    #     hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # to HSV colorspace
        
    #     color_filt_frame, mask = self.preprocess(hsv_frame) # color filtering

    #     close_frame = cv2.morphologyEx(color_filt_frame, cv2.MORPH_CLOSE, self.kernel) # fill in
    #     dilate_frame = cv2.dilate(close_frame, self.kernel, iterations=3) # make chubby

    #     hsv2bgr_frame = cv2.cvtColor(dilate_frame, cv2.COLOR_HSV2BGR) # change color space to BGR
    #     grayscale_frame = cv2.cvtColor(hsv2bgr_frame, cv2.COLOR_BGR2GRAY) # to grayscale

    #     ret, thresh_frame = cv2.threshold(grayscale_frame, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #     frame_c, frame_contours, frame_heirarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #     filtered_contours = self.filter_contours(frame_contours) # filter the contours based on size

    #     boxes = [cv2.boundingRect(c) for c in filtered_contours] # make boxes around contours
    #     interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]
        
    #     return interest_regions

    '''
    def get_interest_regions(self, frame):
        
        color_filt_frame, mask = self.preprocess(frame) # color filtering

        grayscale_frame = cv2.cvtColor(color_filt_frame, cv2.COLOR_BGR2GRAY) # to grayscale

        ret, thresh_frame = cv2.threshold(grayscale_frame, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        frame_c, frame_contours, frame_heirarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours = self.filter_contours(frame_contours) # filter the contours based on size

        boxes = [cv2.boundingRect(c) for c in filtered_contours] # make boxes around contours
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]
        
        return interest_regions
    '''


    '''
    # new uses conv filtering
    def get_interest_regions(self, frame):
        ''' blur types '''
        #blur_avg = cv2.blur(frame, (5, 5) )
        #blur_gau = cv2.GaussianBlur(frame, (5, 5), 0)
        blur_med = cv2.medianBlur(frame, 5)
        #blur_bi = cv2.bilateralFilter(frame, 9, 75, 160) # WAS 9, 160, 160

        ### USE only where WALL is to the LEFT of the GATE - since slope of wall is negative
        ''' positive slope '''
        #dst_dp = cv2.filter2D(frame, -1, kernel_diag_pos)
        #dst_dp = cv2.filter2D(blur_avg, -1, kernel_diag_pos)
        #dst_dp = cv2.filter2D(blur_gau, -1, kernel_diag_pos)
        dst_dp = cv2.filter2D(blur_med, -1, kernel_diag_pos)
        #dst_dp = cv2.filter2D(blur_bi, -1, kernel_diag_pos)

        ### USE ony where WALL is to the RIGHT of the GATE - since slope of wall is positive
        ''' negative slope '''
        #dst_dn = cv2.filter2D(frame, -1, kernel_diag_neg)
        #dst_dn = cv2.filter2D(blur_avg, -1, kernel_diag_neg)
        #dst_dn = cv2.filter2D(blur_gau, -1, kernel_diag_neg)
        #dst_dn = cv2.filter2D(blur_med, -1, kernel_diag_neg)
        #dst_dn = cv2.filter2D(blur_bi, -1, kernel_diag_neg)

        grayscale_frame = cv2.cvtColor(dst_dp, cv2.COLOR_BGR2GRAY)
        #grayscale_frame = cv2.cvtColor(dst_dn, cv2.COLOR_BGR2GRAY)
        
        ret, thresh_frame = cv2.threshold(grayscale_frame, 127, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        frame_c, frame_contours, frame_heirarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours = self.filter_contours(frame_contours) # filter the contours based on size

        boxes = [cv2.boundingRect(c) for c in filtered_contours] # make boxes around contours
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]

        return interest_regions
        '''

    # new BGR color filter - only finds bars right now
    def get_interest_regions(self, frame):
        
        blur_frame = cv2.medianBlur(frame, 9)
        
        color_filt_frame, mask = self.preprocess(blur_frame) # color filtering
        
        grayscale_frame = cv2.cvtColor(color_filt_frame, cv2.COLOR_BGR2GRAY)
        
        ret, thresh_frame = cv2.threshold(grayscale_frame, 127, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        frame_c, frame_contours, frame_heirarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours = self.filter_contours(frame_contours) # filter the contours based on size

        boxes = [cv2.boundingRect(c) for c in filtered_contours] # make boxes around contours
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]

        return interest_regions

