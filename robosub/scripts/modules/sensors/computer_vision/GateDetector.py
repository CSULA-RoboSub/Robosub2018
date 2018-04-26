import utils
import time
import GateClassifier as gc
import GatePreprocessor as gp
import cv2

class GateDetector:

    def __init__(self):
        self.classifier = gc.GateClassifier()
        self.found =  False;
        self.cap = cv2.VideoCapture(0)
        self.preprocess = gp.GatePreprocessor()
        #self.hog = self.classifier.get_hog() # why - to init? should init in class...
        #self.lsvm = self.classifier.get_lsvm() # why - same?
        self.lower = [40,60,60]
        self.upper = [60,255,255]
        self.directions = [0,0]
        self.isTaskComplete = False
        #print self.isTaskComplete # python2
        print(self.isTaskComplete)

        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('gate-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (640, 480))
        
    '''
    def get_directions(self,x,y,w,h):
        return utils.get_directions(x,y,w,h)
    '''
    
    def detect(self):
        ret, frame = self.cap.read()
        height, width, ch = frame.shape
        center = (width / 2, height / 2)
        regions_of_interest = self.preprocess.get_interest_regions(frame)

        for x, y, w, h in regions_of_interest:
            cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["red"], 2)

        #cv2.imshow('frame', frame)
        gate = self.classifier.classify(frame, regions_of_interest)

        if (gate == None):
            self.directions = [0, 0]
            self.found = False
        else:
            x, y, w, h = gate
            cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["blue"], 6)
            self.directions = utils.get_directions(center, x, y ,w, h)
            self.found = True

        self.out.write(frame)
        
        return self.found, self.directions
