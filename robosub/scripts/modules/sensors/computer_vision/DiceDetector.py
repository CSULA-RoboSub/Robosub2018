'''Dice Detector does the actual detection of all the dice objects and
handles direction setting for the ros to grab'''
import math
import Detector
import utils
import DiceClassifier as dc
import DicePreprocess as dpp
import cv2

class DiceDetector:

    '''
    the initialization will take a cam parameter so that we know which camera we are selecting
        later in documentation we should include which camera is which by number.
    '''
    def __init__(self):
        self.preprocessor = dpp.DicePreprocessor()
        self.classifier = dc.DiceClassifier()
        self.directions = [None, None]
        self.dot_size = 100 
        self.found = False

        self.shapes = {1: "vertical", 2: "horizontal", 3: "square"} # so we can change names quicker
        self.shape_ratio_lower = 0.8
        self.shape_ratio_upper = 1.20
        
        # using die 5 and 6 for the time being since those are the only
        # dies available
        self.die_1 = 5
        self.die_2 = 6
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

    def detect(self,frame, die_num = None):
        if die_num is None:
            die_num = 5

        if frame is not None:
            interest_regions =  self.preprocessor.get_interest_regions(frame)
            # die = [die for die in interest_regions if self.classifier.predict(die) > .1]
            classified_rois = self.classifier.classify(frame, interest_regions)
            dice = self.has_2_neighbors(classified_rois, self.shapes[3])
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
            print('error no frame')
            return False, None, None, None

    def sort_smallest(self, interest_regions):
        #get smallest roi that pass through filter
        # return min(interest_regions, key=lambda x: x[2]*x[3])
        #sort list in place
        interest_regions.sort(key = lambda x: x[2]*x[3])

    def has_2_neighbors(self, interest_regions, shape):
        if interest_regions:
            self.sort_smallest(interest_regions)

            cx, cy, cw, ch = interest_regions[0]
            carea = float(cw) * float(ch)
            neighbor_area_buffer = carea * 0.2
            area_check_upper = carea + neighbor_area_buffer
            area_check_lower = carea - neighbor_area_buffer

            neighbor_range = float(max(cw, ch))
            neighbor_range_buffer = neighbor_range * 2.5

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
                        counted_rois.pop(0)
                        counted_rois.append(interest_regions[i])
                        cx = x
                        cy = y
                        cw = w
                        ch = h 
                try:
                    min_x = min(counted_rois, key=lambda x: x[0])[0]
                    min_y = min(counted_rois, key=lambda x: x[1])[1]

                    max_x = max(counted_rois, key=lambda x: x[0])[0]
                    max_y = max(counted_rois, key=lambda x: x[1])[1]
                    
                    max_w = max(counted_rois, key=lambda x: x[0]+x[2])[2]
                    max_h = max(counted_rois, key=lambda x: x[1]+x[3])[3]
                except:
                    return None

                if neighbor_count > 2:
                    w_ret = max_x - min_x + max_w
                    h_ret = max_y - min_y + max_h
                    return min_x, min_y, w_ret, h_ret 
            
        return None
    # def change_dice(self):
    #     print 'changing die number from die1 to die2 here'
    #     # TODO need to implement way for changing number here

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
