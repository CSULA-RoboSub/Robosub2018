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

    def detect(self,frame):
        interest_regions =  self.preprocessor.get_interest_regions(frame)
        die = [die for die in interest_regions if self.classifier.predict(die) > .1]

        ht , wd =  frame.shape
        if dice == None:
            found = False
            dice_shape = None
            self.directions = [0,0]
            w,h = 0,0
        else:
            x, y, w, h  = die
            dice_shape = self.get_shape(die, self.shape_buffer)
            self.directions = utils.get_directions( (wd/2, ht/2), x, y, w, h) 
            self.found = True
            
        #found, direction, shape, width, heightk
        # return (self.found, self.directions, None, (0, 0)) 
        return (self.found, self.directions, dice_shape, (w, h))

    def search_die(self, frame, value):
        found = False
        dice = self.locate_dice(frame)
        for die in dice:
            if self.get_dots(die) == value:
                found = True
                self.directions = utils.get_directions(die)
        return found, self.directions

    def get_dots(self, die):
        _,bw_die = cv2.cvtColor(die, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(bw_die, 127, 255, 0)
        temp_im, conts, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        dots = [cv2.boundingRect(c) for c in conts]
        dots = [d for d in dots if d[2] * d[3] < self.dot_size]

        return len(dots)
