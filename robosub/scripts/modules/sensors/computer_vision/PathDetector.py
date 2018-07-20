import utils
import time
import PathClassifier as pc
import PathPreprocessor as pp
import cv2

class PathDetector():

    def __init__(self):
        self.classifier = pc.PathClassifier()
        self.found = False
        self.preprocess = pp.PathPreprocessor()
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

    def detect(self, frame):
        height, width, ch = frame.shape
        center = (width / 2, height / 2)
        regions_of_interest = self.preprocess.get_interest_regions(frame)
        
        for x, y, w, h in regions_of_interest:
            cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["red"], 2)

        # path = self.classifier.classify(frame, regions_of_interest)
        if regions_of_interest:
            path = regions_of_interest[0]
        else:
            path = None
            
        path_shape = self.get_shape(path, self.shape_buffer)
        
        if (path == None):
            self.directions = [0, 0]
            self.found = False
            w, h = 0, 0
        else:
            x, y, w, h = path
            cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["blue"], 6)
            self.directions = utils.get_directions(center, x, y, w, h)
            self.found = True
        return (self.found, self.directions, path_shape, (w, h))