import utils
import time
import BuoyClassifier as bc
import cv2
import BuoyPreprocessor as bp


class BuoyDetector:

    def __init__(self):
        self.classifier = bc.BuoyClassifier()
        self.found =  False;
        self.preprocess = bp.BuoyPreprocessor()
        self.hog = self.classifier.get_hog()
        self.lsvm = self.classifier.get_lsvm()
        print (self.lsvm)
        self.lower = [40,60,60]
        self.upper = [60,255,255]
        self.directions = [0,0]
        self.isTaskComplete = False
        print self.isTaskComplete

        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(str(time.time()) + 'output.avi', self.fourcc, 20.0, (640, 480))
    '''
    def get_directions(self,x,y,w,h):
        return utils.get_directions(x,y,w,h)
    '''
    def detect(self):
        _, frame = self.cap.read()
        height, width, _ = frame.shape
        center = (width / 2, height / 2)
        regions_of_interest = self.preprocess.get_interest_regions(frame)

        '''
        for x, y, w, h in regions_of_interest:
            cv2.rectangle(frame, (x,y), (x+w, y+h), utils.colors["red"], 2)
        '''
        #clone = frame.copy()

        #cv2.imshow('frame',frame)
        buoy = self.classifier.classify(frame, regions_of_interest)

        if buoy == None:
            self.directions = [0,0]
            self.found = False
        else:
            x,y,w,h = buoy
            cv2.rectangle(frame, (x,y), (x+w, y+h), utils.colors["blue"], 6)
            self.directions = utils.get_directions(center, x,y,w,h)
            self.found = True

        self.out.write(frame)
        return self.found, self.directions

