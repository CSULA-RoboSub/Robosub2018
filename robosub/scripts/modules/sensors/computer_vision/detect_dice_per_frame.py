import os

import cv2
import matplotlib.pyplot as plt
import numpy as np

class DetectDicePerFrame:
    def __init__(self):
        self.sensitivity = 10
        self.lower = np.array([0,0,100])
        self.upper = np.array([255,self.sensitivity,255])

        self.min_cont_size = 100
        self.max_cont_size = 1000

    def dilate(self, img, iterations=1):
        kernel = np.ones((5,5), np.uint8)
        return cv2.dilate(img, kernel, iterations=iterations)

    def preprocess(self, img):
        die = 5
    #     if die is None:
    #         die = 5
    #     elif die != 5 and die != 6:
    #         return None

        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, self.lower, self.upper)
        color_filter_frame = cv2.bitwise_and(img, img, mask=mask)
        grayscale_frame = cv2.cvtColor(color_filter_frame, cv2.COLOR_BGR2GRAY) 
        dilated_frame = self.dilate(grayscale_frame, iterations=1)
        # to grayscale
        return grayscale_frame, mask

    def filter_contours(self, frame_contours):
        new_cont_list = []
        for cont in frame_contours:
            cont_len = len(cont)
            if self.min_cont_size < cont_len < self.max_cont_size:
                new_cont_list.append(cont)
        filtered_contours = np.array(new_cont_list)
        return filtered_contours


    def get_interest_regions(self, frame, die=None): 
        frame_c, frame_contours, frame_hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_filtered_contours = self.filter_contours(frame_contours)   
        boxes = [cv2.boundingRect(c) for c in frame_contours]
        return boxes

    def get_bounding_boxes(self, frame):
        boxes = self.get_interest_regions(self.preprocess(frame)[0])
        return [b for b in boxes if b[2]*b[3] > 300 and b[2] < 300 and b[3] < 300]

    def draw_boxes_on_dice(self, frame):
        boxes = self.get_bounding_boxes(frame)
        for x, y, w, h in boxes:
            cv2.rectangle(frame, (x,y),(x+w, y+h), (0, 255, 0), 2)

    def draw_max_box_on_dice(self, frame):
        x, y, w, h = self.get_bounding_box_with_max_pips(frame)
        cv2.rectangle(frame, (x,y),(x+w, y+h), (0, 255, 0), 2)

    def draw_second_max_box_on_dice(self, frame):
        x, y, w, h = self.get_bounding_box_with_second_most_pips(frame)
        cv2.rectangle(frame, (x,y),(x+w, y+h), (0, 255, 0), 2)

    def find_pips(self, img):
        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create()

        # Detect blobs.
        keypoints = detector.detect(img)

        return keypoints

    def get_crop_from_bounding_box(self, img, box):
        x, y, w, h = box
        return img[y:y+h, x:x+w]

    def get_bounding_box_binary_images(self, img, boxes):
        binary = self.preprocess(img.copy())[0]
        imgs = [(binary[y:y+h,x:x+w], (x,y,w,h)) for (x, y, w, h) in boxes]
        return imgs

    def get_bounding_box_with_max_pips(self, img):
        boxes = self.get_bounding_boxes(img)
        if not boxes or boxes is None:
            return (0, 0, 0, 0)

        max_box = max(boxes, key=lambda box: len(self.find_pips(self.get_crop_from_bounding_box(img, box))))
        pips = self.find_pips(self.get_crop_from_bounding_box(img, max_box))
        if len(pips) > 4:
            return max_box
        return (0,0,0,0)

    def get_bounding_box_with_second_most_pips(self, img):
        boxes = self.get_bounding_boxes(img)
        #need at least two boxes
        if len(boxes) < 2:
            return 0,0,0,0
        max_box = max(boxes, key=lambda box: len(self.find_pips(self.get_crop_from_bounding_box(img, box))))
        boxes.remove(max_box)
        
        #do it again with max removed to get second max
        max_box = max(boxes, key=lambda box: len(self.find_pips(self.get_crop_from_bounding_box(img, box))))
        pips = self.find_pips(self.get_crop_from_bounding_box(img, max_box))
        if len(pips) > 4:
            return max_box
        return (0,0,0,0)
