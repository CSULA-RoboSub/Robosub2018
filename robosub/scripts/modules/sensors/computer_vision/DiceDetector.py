'''Dice Detector does the actual detection of all the dice objects and
handles direction setting for the ros to grab'''
import math
import Detector
import utils
import DiceClassifier as dc
# import DicePreprocess as dpp
from detect_dice_per_frame import DetectDicePerFrame as dppf
import cv2

class DiceDetector:

    '''
    the initialization will take a cam parameter so that we know which camera we are selecting
        later in documentation we should include which camera is which by number.
    '''
    def __init__(self):
        # self.preprocessor = dpp.DicePreprocessor()
        self.preprocessor = dppf()
        self.is_dppf = True

        self.classifier = dc.DiceClassifier()
        self.directions = [None, None]
        self.dot_size = 100 
        self.found = False

        self.shapes = {1: "vertical", 2: "horizontal", 3: "square"} # so we can change names quicker
        self.shape_ratio_lower = 0.76
        self.shape_ratio_upper = 1.24
        
        self.die = 5
        self.frame_size = (744, 480)

        self.dies = {
            0: 5,
            1: 6
        }
        self.die_num = 0

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

    def detect(self, frame):
        if frame is not None:
            if not self.is_dppf:
                interest_regions =  self.preprocessor.get_interest_regions(frame, self.die)
                # die = [die for die in interest_regions if self.classifier.predict(die) > .1]
                classified_rois = self.classifier.classify(frame, interest_regions)
                
                if self.die == 5:
                    dice = self.detect_five(classified_rois, self.shapes[3])
                elif self.die == 6:
                    dice = self.detect_six(classified_rois, self.shapes[1], self.shapes[2])
                else:
                    return False, None, None, None
                #TODO we need to implement a way to use self.die_1 and self.die_2
                # in the detect to return the right coordinates. we can check if the die
                # is touched by the sub if the frame is equal to the region of interest (744x480)
                # or (640x480) for the laptop camera

                # we can also use the dictionary, whichever one is easier

                for x, y, w, h in classified_rois:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["red"], 2)

                ht, wd, ch =  frame.shape

                dice_shape = self.get_shape(dice)
                # if dice_shape != self.shapes[1]:
                #     dice = None
                    
                if not dice:
                    self.found = False
                    dice_shape = None
                    self.directions = [0,0]
                    w,h = 0,0
                else:
                    x, y, w, h  = dice
                    # dice_shape = self.get_shape(dice, self.shape_buffer)
                    # dice_shape = None
                    cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["blue"], 6)
                    self.directions = utils.get_directions( (wd/2, ht/2), x, y, w, h) 
                    self.found = True

                #found, direction, shape, width, heightk
                # return (self.found, self.directions, None, (0, 0)) 
                return (self.found, self.directions, dice_shape, (w, h))
            else:
                # interest_regions = self.preprocessor.get_bounding_boxes(frame)

                if self.die == 5:
                    dice = self.preprocessor.get_bounding_box_with_second_most_pips(frame)
                elif self.die == 6:
                    dice = self.preprocessor.get_bounding_box_with_max_pips(frame)
                else:
                    return False, None, None, None

                # for x, y, w, h in interest_regions:
                #     cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["red"], 2)

                # self.preprocessor.draw_max_box_on_dice(frame)
                # self.preprocessor.draw_second_max_box_on_dice(frame)
                ht, wd, ch =  frame.shape
                # dice = utils.get_max_area(interest_regions)
                dice_shape = self.get_shape(dice)
                # if dice_shape != self.shapes[1]:
                #     dice = None
                    
                if not dice:
                    self.found = False
                    dice_shape = None
                    self.directions = [0,0]
                    w,h = 0,0
                else:
                    x, y, w, h  = dice
                    # dice_shape = self.get_shape(dice, self.shape_buffer)
                    # dice_shape = None
                    cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["blue"], 6)
                    self.directions = utils.get_directions( (wd/2, ht/2), x, y, w, h) 
                    self.found = True

                #found, direction, shape, width, heightk
                # return (self.found, self.directions, None, (0, 0)) 
                return (self.found, self.directions, dice_shape, (w, h))
        else:
            print('error no frame')
            return False, None, None, None

    def sort_smallest(self, interest_regions):
        #get smallest roi that pass through filter
        # return min(interest_regions, key=lambda x: x[2]*x[3])
        #sort list in place
        interest_regions.sort(key = lambda x: x[2]*x[3])

    def sort_largest(self, interest_regions):
        #get smallest roi that pass through filter
        # return min(interest_regions, key=lambda x: x[2]*x[3])
        #sort list in place
        interest_regions.sort(key = lambda x: x[2]*x[3], reverse = True)

    def detect_five(self, interest_regions, shape):
        if interest_regions:
            self.sort_smallest(interest_regions)

            area_mult = 0.2
            range_mult = 2.5

            cx, cy, cw, ch = interest_regions[0]
            carea = float(cw) * float(ch)
            neighbor_area_buffer = carea * area_mult
            area_check_upper = carea + neighbor_area_buffer
            area_check_lower = carea - neighbor_area_buffer

            neighbor_range = float(max(cw, ch))
            neighbor_range_buffer = neighbor_range * range_mult

            neighbor_count = 0
            counted_rois = []
            counted_rois.append(interest_regions[0])

            len_rois = len(interest_regions)
            if len_rois > 2:
                for i in range(1, len_rois):
                    x, y, w, h = interest_regions[i]
                    if (cx-neighbor_range_buffer <= x <= cx+neighbor_range_buffer) and (cy-neighbor_range_buffer <= y <= cy+neighbor_range_buffer) and (area_check_lower <= (w*h) <= area_check_upper) and self.get_shape(interest_regions[i]) == shape:
                        neighbor_count += 1
                        counted_rois.append(interest_regions[i])

                    elif neighbor_count < 1:
                        cx, cy, cw, ch = interest_regions[i]
                        carea = float(cw) * float(ch)
                        neighbor_area_buffer = carea * area_mult
                        area_check_upper = carea + neighbor_area_buffer
                        area_check_lower = carea - neighbor_area_buffer

                        neighbor_range = float(max(cw, ch))
                        neighbor_range_buffer = neighbor_range * range_mult

                        neighbor_count = 0
                        counted_rois = []
                        counted_rois.append(interest_regions[i])

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

                    if max_x+max_w < cr[0] + cr[2]:
                        max_x = cr[0]
                        max_w = cr[2]
                    if max_y+max_h < cr[1] + cr[3]:
                        max_y = cr[1]
                        max_h = cr[3]

                # try:
                #     min_x = min(counted_rois, key=lambda x: x[0])[0]
                #     min_y = min(counted_rois, key=lambda x: x[1])[1]

                #     max_x = max(counted_rois, key=lambda x: x[0])[0]
                #     max_y = max(counted_rois, key=lambda x: x[1])[1]

                #     max_w = max(counted_rois, key=lambda x: x[0]+x[2])[2]
                #     max_h = max(counted_rois, key=lambda x: x[1]+x[3])[3]
                # except:
                #     return None

                if neighbor_count > 2:
                    w_ret = max_x - min_x + max_w
                    h_ret = max_y - min_y + max_h
                    return min_x, min_y, w_ret, h_ret 
            
        return None

    def detect_six(self, interest_regions, shape1, shape2):
        if interest_regions:
            self.sort_largest(interest_regions)

            area_mult = 0.2
            range_mult = 1.5

            cx, cy, cw, ch = interest_regions[0]
            carea = float(cw) * float(ch)
            neighbor_area_buffer = carea * area_mult
            area_check_upper = carea + neighbor_area_buffer
            area_check_lower = carea - neighbor_area_buffer

            neighbor_range = float(max(cw, ch))
            neighbor_range_buffer = neighbor_range * range_mult

            neighbor_count = 0
            counted_rois = []
            counted_rois.append(interest_regions[0])

            len_rois = len(interest_regions)
            if len_rois > 1:
                for i in range(1, len_rois):
                    x, y, w, h = interest_regions[i]
                    shape_i = self.get_shape(interest_regions[i])
                    if (cx-neighbor_range_buffer <= x <= cx+neighbor_range_buffer) and (cy-neighbor_range_buffer <= y <= cy+neighbor_range_buffer) and (area_check_lower <= (w*h) <= area_check_upper) and (shape_i == shape1 or shape_i == shape2):
                        neighbor_count += 1
                        counted_rois.append(interest_regions[i])

                    elif neighbor_count < 1:
                        cx, cy, cw, ch = interest_regions[i]
                        carea = float(cw) * float(ch)
                        neighbor_area_buffer = carea * area_mult
                        area_check_upper = carea + neighbor_area_buffer
                        area_check_lower = carea - neighbor_area_buffer

                        neighbor_range = float(max(cw, ch))
                        neighbor_range_buffer = neighbor_range * range_mult

                        neighbor_count = 0
                        counted_rois = []
                        counted_rois.append(interest_regions[i])

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

                    if max_x+max_w < cr[0] + cr[2]:
                        max_x = cr[0]
                        max_w = cr[2]
                    if max_y+max_h < cr[1] + cr[3]:
                        max_y = cr[1]
                        max_h = cr[3]
                # try:
                #     min_x = min(counted_rois, key=lambda x: x[0])[0]
                #     min_y = min(counted_rois, key=lambda x: x[1])[1]

                #     max_x = max(counted_rois, key=lambda x: x[0])[0]
                #     max_y = max(counted_rois, key=lambda x: x[1])[1]

                #     max_w = max(counted_rois, key=lambda x: x[0]+x[2])[2]
                #     max_h = max(counted_rois, key=lambda x: x[1]+x[3])[3]
                # except:
                #     return None

                if neighbor_count >= 1:
                    w_ret = max_x - min_x + max_w
                    h_ret = max_y - min_y + max_h
                    return min_x, min_y, w_ret, h_ret 
            
        return None

    def change_dice(self, die):
        if die != 5 or die != 6:
            return

        self.die = die

    # def search_die(self, frame, value):
    #     found = False
    #     dice = self.locate_dice(frame)
    #     for die in dice:
    #         if self.get_dots(die) == value:
    #             found = True
    #             self.directions = utils.get_directions(die)
    #     return found, self.directions

    # def get_dots(self, die):
    #     _,bw_die = cv2.cvtColor(die, cv2.COLOR_BGR2GRAY)
    #     _, th = cv2.threshold(bw_die, 127, 255, 0)
    #     temp_im, conts, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #     dots = [cv2.boundingRect(c) for c in conts]
    #     dots = [d for d in dots if d[2] * d[3] < self.dot_size]

    #     return len(dots)
