import utils
import time
import GateClassifier as gc
import GatePreprocessor as gp
import cv2

class GateDetector:

    def __init__(self):
        self.classifier = gc.GateClassifier()
        self.found =  False;
        self.preprocess = gp.GatePreprocessor()
        self.directions = [0,0]
        self.isTaskComplete = False
        self.shapes = {1: "vertical", 2: "horizontal", 3: "square"} # so we can change names quicker
        self.shape_buffer = 15
        self.shape_list = []

    # takes a single-roi coordinate as (x, y, w, h) and a buffer as an int
    # returns the shape as a string
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

    # now returns (found, directions, shape-of-roi, size)
    def detect(self, frame):
        height, width, ch = frame.shape
        center = (width / 2, height / 2)
        regions_of_interest = self.preprocess.get_interest_regions(frame)
        
        for x, y, w, h in regions_of_interest:
            cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["red"], 2)

        gate = self.classifier.classify(frame, regions_of_interest)
        
        gate_shape = self.get_shape(gate, self.shape_buffer)

        if gate_shape == self.shapes[3] or gate_shape == self.shapes[1]:
            gate = None
            
        if (gate == None):
            self.directions = [0, 0]
            self.found = False
            gate_shape = None
            w, h = 0, 0
        else:
            x, y, w, h = gate
            cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["blue"], 6)
            self.directions = utils.get_directions(center, x, y, w, h)
            self.found = True
        return (self.found, self.directions, gate_shape, (w, h))
