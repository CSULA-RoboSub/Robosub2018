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
        self.shape_buffer = 15
        
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

    def get_shape(self, roi, buff):
        if roi == None:
            return None
        else:
            x, y, w, h = roi

        #if ( (h >= (w + buff) ) or (h >= (w - buff) )):
        if h - w > buff:
            return self.shapes[1] # vertical
        #elif ( (h <= (w + buff) ) or (h <= (w - buff) )):
        elif w - h > buff:    
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

            dice_shape = self.get_shape(dice, self.shape_buffer)
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

            smallest = interest_regions[0]
            neighbor_area_buffer = smallest[2]*smallest[3] * 0.4
            area_check_upper = smallest[2] * smallest[3] + neighbor_area_buffer
            area_check_lower = smallest[2] * smallest[3] - neighbor_area_buffer
            #compare
            neighbor_range = max(smallest[2], smallest[3])
            neighbor_range_buffer = neighbor_range * 2.0
            n_range_check = neighbor_range + neighbor_range_buffer

            neighbor_count = 0

            left_top_corner = list(smallest)
            right_bottom_corner = list(smallest)
            min_x = smallest[0]
            min_y = smallest[1]
            max_x = smallest[0]+smallest[2]
            max_y = smallest[1]+smallest[3]
            max_w = smallest[2]
            max_h = smallest[3]
            counted_rois = []

            if len(interest_regions) > 1:
                for i in range(1, len(interest_regions)):
                    x, y, w, h = interest_regions[i]
                    if (smallest[0] - n_range_check <= x <= smallest[0] + n_range_check) and (smallest[1] - n_range_check <= y <= smallest[1] + n_range_check) and (area_check_lower <= (w*h) <= area_check_upper) and self.get_shape(interest_regions[i], self.shape_buffer) == shape:
                        neighbor_count += 1
                        counted_rois.append(interest_regions[i])
                        #find min x,y for top left corner
                        # if min_x < x:
                        #     min_x = x
                        
                        # if min_y < y:
                        #     min_y = y

                        # #find max x+w and y+h for bottom right corner
                        # if max_x > x+w:
                        #     max_x = x+w

                        # if max_y > y+h:
                        #     max_y = y+h

                        # if max_w > w:
                        #     max_w = w

                        # if max_h > h:
                        #     max_h = h
                try:
                    min_x = min(counted_rois, key=lambda x: x[0])[0]
                    min_y = min(counted_rois, key=lambda x: x[1])[1]
                    max_x = max(counted_rois, key=lambda x: x[0])[0]
                    max_y = max(counted_rois, key=lambda x: x[1])[1]
                    max_w = max(counted_rois, key=lambda x: x[2])[2]
                    max_h = max(counted_rois, key=lambda x: x[3])[3]
                except:
                    return None

                w_fin = max_x - min_x
                h_fin = max_y - min_y

                if neighbor_count >= 2:
                    w_ret = max_x - min_x + w_fin
                    h_ret = max_y - min_y + h_fin
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
