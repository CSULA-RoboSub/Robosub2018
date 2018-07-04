import utils
import time
import GateClassifier as gc
import GatePreprocessor as gp
import cv2

class GateDetector:

    def __init__(self):
        self.classifier = gc.GateClassifier()
        self.found =  False;
#        self.cap = cv2.VideoCapture(0)
        self.preprocess = gp.GatePreprocessor()
        self.directions = [0,0]
        self.isTaskComplete = False
        self.shapes = {1: "vertical", 2: "horizontal", 3: "square"} # so we can change names quicker
        self.shape_buffer = 15
        self.shape_list = []
        
        #print self.isTaskComplete # python2
        #print(self.isTaskComplete)
#       self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
#       self.out = cv2.VideoWriter('gate-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (640, 480))

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
            # have idea to use get_shape here so we classify only squares?
            # add each to self.shape_list? - good for testing
            # roi = (x, y, w, h)
            # self.shape_list.add(roi) or
            # gate_rois = self.classifier.classify(frame, roi) etc..

        #cv2.imshow('frame', frame)
        gate = self.classifier.classify(frame, regions_of_interest)
        
        gate_shape = self.get_shape(gate, self.shape_buffer)
        
        if (gate == None):
            self.directions = [0, 0]
            self.found = False
            w, h = 0, 0
        else:
            x, y, w, h = gate
            cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["blue"], 6)
<<<<<<< HEAD
            self.directions = utils.get_directions(center, x, y, w, h)
=======
            self.directions = utils.get_directions( center, x, y, w, h )
>>>>>>> 2055e4f24c543828803d1e45ee1f2d56fe9ecf71
            self.found = True
        return (self.found, self.directions, gate_shape, (w, h))