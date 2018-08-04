import utils
import cv2
import numpy as np

class PathPreprocessor():

    def __init__(self):
        #bright 1
        # self.lower_thresh = np.array([0, 109, 5], 'uint8')
        # self.upper_thresh = np.array([18, 255, 255], 'uint8')
        # self.lower_thresh = np.array([0, 49, 39], 'uint8')
        # self.upper_thresh = np.array([18, 255, 255], 'uint8')

        self.lower_red_orange = np.array([10, 2, 66], 'uint8')
        self.upper_red_orange = np.array([60, 255, 255], 'uint8')

        self.lower_red_blue = np.array([160, 2, 66], 'uint8')
        self.upper_red_blue = np.array([180, 255, 255], 'uint8')

        self.roi_size = 2200
        self.ratio_threshold = 0.50
        self.min_cont_size = 100 # min contours size
        self.max_cont_size = 2000 # max contours size
        self.kernel = np.ones( (5, 5), np.uint8) # basic filter
        self.lower_bgr = np.array([200, 100, 100], 'uint8')
        self.upper_bgr = np.array([255, 255, 255], 'uint8')

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
        # mask = cv2.inRange(img, self.lower_thresh, self.upper_thresh)
        mask_red_orange = cv2.inRange(img, self.lower_red_orange, self.upper_red_orange)
        mask_red_blue = cv2.inRange(img, self.lower_red_blue, self.upper_red_blue)
        mask = cv2.bitwise_or(mask_red_orange, mask_red_blue)
            
        # mask_preblur = cv2.addWeighted(mask_red_orange, 1.0, mask_red_blue, 1.0, 0)
        # mask = cv2.GaussianBlur(mask_preblur,(9,9),0)

        output = cv2.bitwise_and(img, img, mask = mask)

        return output, mask

    def get_interest_regions(self, frame):

        mask_bgr = cv2.inRange(frame, self.lower_bgr, self.upper_bgr)
        bgr_frame = cv2.bitwise_and(frame, frame, mask=mask_bgr)

        img_hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        img_color_filt, mask = self.preprocess(img_hsv)

        img_bgr = cv2.cvtColor(img_color_filt, cv2.COLOR_HSV2BGR)
        
        open_frame = cv2.morphologyEx(img_bgr, cv2.MORPH_OPEN, self.kernel) # fill in

        # blur_frame = cv2.GaussianBlur(open_frame,(7,7),0)
        blur_frame = cv2.medianBlur(open_frame, 7)

        # dilate_frame = cv2.dilate(blur_frame, self.kernel, iterations=5) # make chubby
        # close_frame = cv2.morphologyEx(blur_frame, cv2.MORPH_CLOSE, self.kernel) # fill in
        # open_frame = cv2.morphologyEx(close_frame, cv2.MORPH_OPEN, self.kernel) # fill in

        bgr_from_hsv = cv2.cvtColor(blur_frame, cv2.COLOR_HSV2BGR)
        grayscale_frame = cv2.cvtColor(bgr_from_hsv, cv2.COLOR_RGB2GRAY)
        # clahe = cv2.createCLAHE(clipLimit=10.0, tileGridSize=(8,8))
        # clahe_applied = clahe.apply(grayscale_frame)
        im, contours, hierarchy = cv2.findContours(grayscale_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # filtered_contours = self.filter_contours(contours)
        # boxes = [cv2.boundingRect(c) for c in filtered_contours]
        interest_regions = []
        for c in contours:
            box = cv2.boundingRect(c)
            if (box[2]*box[3] > self.roi_size) and (cv2.contourArea(c)/box[2]*box[3] > self.ratio_threshold):
                interest_regions.append(box)
        # boxes = [cv2.boundingRect(c) for c in contours]
        # interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]

        return interest_regions, contours


            # mask_bgr = cv2.inRange(img, self.lower_bgr, self.upper_bgr)
            # bgr_frame = cv2.bitwise_and(img, img, mask=mask_bgr)

            # hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV) # to HSV colorspace

            # mask_red_orange = cv2.inRange(hsv_frame, self.lower_red_orange, self.upper_red_orange)
            # mask_red_blue = cv2.inRange(hsv_frame, self.lower_red_blue, self.upper_red_blue)
            # mask = cv2.addWeighted(mask_red_orange, 1.0, mask_red_blue, 1.0, 0)

            # color_filt_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)        

            # close_frame = cv2.morphologyEx(color_filt_frame, cv2.MORPH_CLOSE, self.kernel) # fill in
            # dilate_frame = cv2.dilate(close_frame, self.kernel, iterations=3) # make chubby
            # output = cv2.cvtColor(dilate_frame, cv2.COLOR_HSV2BGR) # change color space to BGR
