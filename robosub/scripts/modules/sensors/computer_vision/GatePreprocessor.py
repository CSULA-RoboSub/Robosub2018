import utils
import cv2
import numpy as np
import pandas as pd
from sklearn.neighbors import NearestNeighbors
from modules.sensors.computer_vision.utilities.filters import *

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
        self.lower_red_orange = np.array([0, 30, 29], 'uint8')
        self.upper_red_orange = np.array([81, 255, 255], 'uint8')

        self.lower_red_blue = np.array([130, 30, 29], 'uint8')
        self.upper_red_blue = np.array([180, 255, 255], 'uint8')

        ''' BGR color values '''
        #self.lower_bgr = np.array([212, 0, 0], 'uint8')
        #self.upper_bgr = np.array([200, 255, 255], 'uint8')

        self.lower_bgr = np.array([130, 150, 210], 'uint8')
        self.upper_bgr = np.array([240, 255, 255], 'uint8')

        # transdec
        self.lower_hsv = np.array([0, 40, 150], 'uint8')
        self.upper_hsv = np.array([10, 255, 255], 'uint8')

        #self.lower_bgr = np.array([130, 150, 190], 'uint8')
        #self.upper_bgr = np.array([254, 255, 255], 'uint8')

        #----------------------------------------------------------------
        #flags only enable one
        self.use_bgr = True
        self.use_hsv_and_bgr = False
        self.use_bgr_and_hsv = False
        self.use_hsv2bgr = False
        #-------------------------------------------------

        self.min_cont_size = 100 # min contours size
        self.max_cont_size = 1000 # max contours size
        self.roi_size = 1000 # box size
        self.morph_ops = True # testing

        ''' KERNEL '''
        self.kernel_dil = np.ones( (5, 5), np.uint8) # basic filter
        self.kernel = kernel_diag_pos
        self.shapes = {1: "vertical", 2: "horizontal", 3: "square"} # so we can change names quicker
        self.shape_buffer = 15
        self.frame_size = (744, 480)
        self.shape_ratio_lower = 0.20
        self.shape_ratio_upper = 1.80

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

        elif self.use_hsv2bgr:
            mask_hsv = cv2.inRange(img, self.lower_hsv, self.upper_hsv)
            output_hsv = cv2.bitwise_and(img, img, mask=mask_hsv)

            hsv2bgr = cv2.cvtColor(output_hsv, cv2.COLOR_HSV2BGR)

            mask = cv2.inRange(hsv2bgr, self.lower_bgr, self.upper_bgr)
            output = cv2.bitwise_and(img, img, mask=mask)

        elif self.use_hsv_and_bgr:
            hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # to HSV colorspace

            mask_red_orange = cv2.inRange(hsv_frame, self.lower_red_orange, self.upper_red_orange)
            mask_red_blue = cv2.inRange(hsv_frame, self.lower_red_blue, self.upper_red_blue)
            mask_hsv = cv2.bitwise_or(mask_red_orange, mask_red_blue)

            color_filt_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask_hsv)

            close_frame = cv2.morphologyEx(color_filt_frame, cv2.MORPH_CLOSE, self.kernel_dil) # fill in
            dilate_frame = cv2.dilate(close_frame, self.kernel_dil, iterations=3) # make chubby
            output_hsv = cv2.cvtColor(dilate_frame, cv2.COLOR_HSV2BGR) # change color space to BGR

            mask = cv2.inRange(output_hsv, self.lower_bgr, self.upper_bgr)
            output = cv2.bitwise_and(output_hsv, output_hsv, mask=mask)

        elif self.use_bgr_and_hsv:
            mask_bgr = cv2.inRange(img, self.lower_bgr, self.upper_bgr)
            bgr_frame = cv2.bitwise_and(img, img, mask=mask_bgr)

            hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV) # to HSV colorspace

            mask_red_orange = cv2.inRange(hsv_frame, self.lower_red_orange, self.upper_red_orange)
            mask_red_blue = cv2.inRange(hsv_frame, self.lower_red_blue, self.upper_red_blue)
            mask = cv2.bitwise_or(mask_red_orange, mask_red_blue)

            color_filt_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)

            close_frame = cv2.morphologyEx(color_filt_frame, cv2.MORPH_CLOSE, self.kernel_dil) # fill in
            dilate_frame = cv2.dilate(close_frame, self.kernel_dil, iterations=3) # make chubby
            output = cv2.cvtColor(dilate_frame, cv2.COLOR_HSV2BGR) # change color space to BGR

        else:
            hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # to HSV colorspace

            mask_red_orange = cv2.inRange(hsv_frame, self.lower_red_orange, self.upper_red_orange)
            mask_red_blue = cv2.inRange(hsv_frame, self.lower_red_blue, self.upper_red_blue)
            mask = cv2.bitwise_or(mask_red_orange, mask_red_blue)

            color_filt_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)

            close_frame = cv2.morphologyEx(color_filt_frame, cv2.MORPH_CLOSE, self.kernel_dil) # fill in
            dilate_frame = cv2.dilate(close_frame, self.kernel_dil, iterations=3) # make chubby
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


    def create_dataset(self, contours):
        if contours is None:
            return None, None
        X = []
        y = []

        for cont in contours:
            if cont is not None:
                M = cv2.moments(cont)
                #cy = int(M["m01"] / M["m00"] + 1) # add 1 to avoid division by zero
                cy = int(M["m01"] / M["m00"] + 1) # add 1 to avoid division by zero
                perimeter = cv2.arcLength(cont, True)
                X.append([perimeter, cy])

        return (pd.DataFrame(X), None )


    def nearest_neighbors(self, dataset, distance=False):
        if dataset is None:
            return None
        elif len(dataset) < 2:
            return None
        else:
            nn = NearestNeighbors(n_neighbors=2)
            nn.fit(dataset)
        return nn.kneighbors(dataset, return_distance=distance)


    # need to error check - converts nearest neighbor list to a better formated list for parsing
    def create_pairs(self, conts):
        if conts is None:
            return None
        new_list = []
        for i in conts:
            tmp_list = []
            tmp_list.append(i[0])
            tmp_list.append(i[1])
            new_list.append(tmp_list)
        return new_list


    def filter_contours(self, frame_contours):
        new_cont_list = []
        for cont in frame_contours:
            cont_len = len(cont)
            if ( (cont_len > self.min_cont_size) and (cont_len < self.max_cont_size) ):
                new_cont_list.append(cont)
        filtered_contours = np.array(new_cont_list)
        return filtered_contours


    def return_box_pairs(self, filtered_contours, converted_pairs):
        if converted_pairs is None:
            return None
        pair_tuples = []
        counter = 0
        for pair in converted_pairs:
            first = filtered_contours[pair[0]]
            second = filtered_contours[pair[1]]
            first_box = cv2.boundingRect(first)
            second_box = cv2.boundingRect(second)
            pair_tuples.append((first_box, second_box))
        return pair_tuples


    def get_interest_regions(self, frame):

        color_filt_frame, mask = self.preprocess(frame) # color filtering

        grayscale_frame = cv2.cvtColor(color_filt_frame, cv2.COLOR_BGR2GRAY) # to grayscale

        ret, thresh_frame = cv2.threshold(grayscale_frame, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        frame_c, frame_contours, frame_heirarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours = self.filter_contours(frame_contours) # filter the contours based on size

        X_df, y_df = self.create_dataset(filtered_contours) # not using y_df

        contour_pairs = self.nearest_neighbors(X_df)

        converted_pairs = self.create_pairs(contour_pairs)

        roi_pairs = self.return_box_pairs(filtered_contours, converted_pairs)

        boxes = self.detect_whole_gate(roi_pairs, self.shapes[1])

        # boxes = [cv2.boundingRect(c) for c in filtered_contours] # make boxes around contours
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size and self.find_number_of_pips(b)>0]

        return interest_regions


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
    # # new uses conv filtering
    # def get_interest_regions(self, frame):'''
    #     ''' blur types '''
    #     #blur_avg = cv2.blur(frame, (5, 5) )
    #     #blur_gau = cv2.GaussianBlur(frame, (5, 5), 0)
    #     blur_med = cv2.medianBlur(frame, 5)
    #     #blur_bi = cv2.bilateralFilter(frame, 9, 75, 160) # WAS 9, 160, 160

    #     ### USE only where WALL is to the LEFT of the GATE - since slope of wall is negative
    #     ''' positive slope '''
    #     #dst_dp = cv2.filter2D(frame, -1, kernel_diag_pos)
    #     #dst_dp = cv2.filter2D(blur_avg, -1, kernel_diag_pos)
    #     #dst_dp = cv2.filter2D(blur_gau, -1, kernel_diag_pos)
    #     dst_dp = cv2.filter2D(blur_med, -1, kernel_diag_pos)
    #     #dst_dp = cv2.filter2D(blur_bi, -1, kernel_diag_pos)

    #     ### USE ony where WALL is to the RIGHT of the GATE - since slope of wall is positive
    #     ''' negative slope '''
    #     #dst_dn = cv2.filter2D(frame, -1, kernel_diag_neg)
    #     #dst_dn = cv2.filter2D(blur_avg, -1, kernel_diag_neg)
    #     #dst_dn = cv2.filter2D(blur_gau, -1, kernel_diag_neg)
    #     #dst_dn = cv2.filter2D(blur_med, -1, kernel_diag_neg)
    #     #dst_dn = cv2.filter2D(blur_bi, -1, kernel_diag_neg)

    #     grayscale_frame = cv2.cvtColor(dst_dp, cv2.COLOR_BGR2GRAY)
    #     #grayscale_frame = cv2.cvtColor(dst_dn, cv2.COLOR_BGR2GRAY)

    #     ret, thresh_frame = cv2.threshold(grayscale_frame, 127, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #     frame_c, frame_contours, frame_heirarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #     filtered_contours = self.filter_contours(frame_contours) # filter the contours based on size

    #     boxes = [cv2.boundingRect(c) for c in filtered_contours] # make boxes around contours
    #     interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size]

    #     return interest_regions
    #     '''

    def find_pips(self, img):
    # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create()
        # Detect blobs.
        keypoints = detector.detect(img)

        return keypoints

    def get_crop_from_bounding_box(self, img, box):
        x, y, w, h = box
        return img[y:y+h, x:x+w]

    def find_number_of_pips(self, box, img):
        crop = self.get_crop_from_bounding_box(img, box)
        return len(self.find_pips(crop))

    

    # new BGR color filter - only finds bars right now
    def get_interest_regions(self, frame):

        bgr2hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_filt_frame, mask = self.preprocess(bgr2hsv) # color filtering

        grayscale_frame = cv2.cvtColor(color_filt_frame, cv2.COLOR_BGR2GRAY)

        ret, thresh_frame = cv2.threshold(grayscale_frame, 127, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        frame_c, frame_contours, frame_heirarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours = self.filter_contours(frame_contours) # filter the contours based on size

        X_df, y_df = self.create_dataset(filtered_contours) # not using y_df

        contour_pairs = self.nearest_neighbors(X_df)

        converted_pairs = self.create_pairs(contour_pairs)

        roi_pairs = self.return_box_pairs(filtered_contours, converted_pairs)

        boxes = self.detect_whole_gate(roi_pairs, self.shapes[1])

        # boxes = [cv2.boundingRect(c) for c in filtered_contours] # make boxes around contours
        interest_regions = [b for b in boxes if b[2]*b[3] > self.roi_size and self.find_number_of_pips(b)>0]

        return interest_regions

    def detect_whole_gate(self, interest_regions, shape):
        ret = []

        if interest_regions:

            area_max = 0
            area_max2 = 0
            len_rois = len(interest_regions)

            if len_rois > 0:
                counted_rois = None
                for i in range(0, len_rois):
                    area_mult = 0.4

                    cx, cy, cw, ch = interest_regions[i][0]
                    cx2, cy2, cw2, ch2 = interest_regions[i][1]

                    carea = float(cw) * float(ch)
                    neighbor_area_buffer = carea * area_mult
                    area_check_upper = carea + neighbor_area_buffer
                    area_check_lower = carea - neighbor_area_buffer
                    carea2 = float(cw2) * float(ch2)
                    neighbor_area_buffer2 = carea2 * area_mult
                    area_check_upper2 = carea2 + neighbor_area_buffer2
                    area_check_lower2 = carea2 - neighbor_area_buffer2

                    # max_carea = max(carea, carea2)

                    if (area_check_lower2 <= (cw*ch) <= area_check_upper2) and self.get_shape(interest_regions[i][0]) == shape and self.get_shape(interest_regions[i][1]) == shape:
                        # neighbor_count += 1
                        counted_rois = interest_regions[0]

                        min_x = self.frame_size[0]
                        min_y = self.frame_size[1]
                        max_x = 0
                        max_y = 0
                        max_w = 0
                        max_h = 0

                        for cr in counted_rois:
                            if min_x > cr[0]:
                                min_x = cr[0]
                            if min_y > cr[1]:
                                min_y = cr[1]
                            if max_x + max_w < cr[0] + cr[2]:
                                max_x = cr[0]
                                max_w = cr[2]
                            if max_y + max_h < cr[1] + cr[3]:
                                max_y = cr[1]
                                max_h = cr[3]

                        if counted_rois is not None:
                            w_ret = max_x - min_x + max_w
                            h_ret = max_y - min_y + max_h

                            ret.append((min_x, min_y, w_ret, h_ret))
                # if len(ret) > 0:
                #     return ret

        return ret


    # def get_shape(self, roi, buff):
    #     if roi == None:
    #         return None
    #     else:
    #         x, y, w, h = roi

    #     #if ( (h >= (w + buff) ) or (h >= (w - buff) )):
    #     if h - w > buff:
    #         return self.shapes[1] # vertical
    #     #elif ( (h <= (w + buff) ) or (h <= (w - buff) )):
    #     elif w - h > buff:
    #         return self.shapes[2] # horizontal
    #     else:
    #         return self.shapes[3] # square


    def get_shape(self, roi, ratio_lower = None, ratio_upper = None):
        if roi == None:
            return None
        else:
            x, y, w, h = roi
            if w == 0 or h == 0:
                return None

        if ratio_lower is None:
            ratio_lower = self.shape_ratio_lower
        if ratio_upper is None:
            ratio_upper = self.shape_ratio_upper

        #if ( (h >= (w + buff) ) or (h >= (w - buff) )):
        if float(w)/float(h) < ratio_lower:
            return self.shapes[1] # vertical
        #elif ( (h <= (w + buff) ) or (h <= (w - buff) )):
        elif float(w)/float(h) > ratio_upper:
            return self.shapes[2] # horizontal
        else:
            return self.shapes[3] # square
