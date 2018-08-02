import os

import cv2
import matplotlib.pyplot as plt
import numpy as np

sensitivity = 10
lower = np.array([0,0,240])
upper = np.array([255,sensitivity,255])


def preprocess(img):
    die = 5
#     if die is None:
#         die = 5
#     elif die != 5 and die != 6:
#         return None

    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower, upper)
    color_filter_frame = cv2.bitwise_and(img, img, mask=mask)
    grayscale_frame = cv2.cvtColor(color_filter_frame, cv2.COLOR_BGR2GRAY) # to grayscale
    return grayscale_frame, mask

def set_lower_color(task_name, lower):
    lower = np.array(lower, 'uint8')
    print 'lower is set to {} for {}'.format(lower, task_name)
    
def set_upper_color(task_name, upper):
    upper = np.array(upper, 'uint8')
    print 'upper is set to {} for {}'.format(upper, task_name)

def filter_contours(frame_contours):
    new_cont_list = []
    for cont in frame_contours:
        cont_len = len(cont)
        if min_cont_size < cont_len < max_cont_size:
            new_cont_list.append(cont)
    filtered_contours = np.array(new_cont_list)
    return filtered_contours


def get_interest_regions(frame, die=None): 
    frame_c, frame_contours, frame_hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_filtered_contours = filter_contours(frame_contours)   
    boxes = [cv2.boundingRect(c) for c in frame_contours]
    return boxes

def get_bounding_boxes(frame):
    boxes = get_interest_regions(preprocess(frame)[0])
    return [b for b in boxes if b[2]*b[3] > 300]

def draw_boxes_on_dice(frame):
    boxes = get_bounding_boxes(frame)
    for x, y, w, h in boxes:
        cv2.rectangle(frame, (x,y),(x+w, y+h), (0, 255, 0), 2)