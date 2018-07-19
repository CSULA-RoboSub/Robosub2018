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

    def detect(self, frame):
        return None, None, None, None