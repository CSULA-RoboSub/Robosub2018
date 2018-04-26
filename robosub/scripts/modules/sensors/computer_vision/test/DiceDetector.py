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
    def __init__(self, cam):
        self.cap = cv2.VideoCapture(cam)
        self.pp = dpp.DicePreprocessor()
        self.classifier = dc.DiceClassifier()
        self.directions = [None, None]
        self.dot_size = 100

    def locate_dice(self):
        return False
    ''''
        find a better way to pair the distances and the value of the dice itself
    '''
    def get_pair(self):
        dice = self.locate_dice()
        dice_dict = {}
        for x, y, w, h in dice:
            dice_dict[(x, y, w, h)] = self.get_dots((x, y, w, h))
        path_centers = []
        for i in range(0,6):
            for j in range (i+1,6):
                if dice_dict[dice[i]] + dice_dict[dice[j]] == 7 or dice_dict[dice[i]] + dice_dict[dice[j]] == 11:
                    path_centers.append((utils.center(dice[i]), utils.center(dice[j])))

        low = 0

        for i in range(1, len(path_centers)):
            if utils.dist(path_centers[i]) < utils.dist(path_centers[low]):
                low = i

    def search_die(self, value):
        candidate_dice = self.pp.get_interest_areas()
        dice = [die for die in candidate_dice if self.classifier.predict(die) > .1]

        for die in dice:
            if self.get_dots(die) == value:
                self.directions = utils.get_directions(die)

    def get_dots(self, die):
        _,bw_die = cv2.cvtColor(die, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(bw_die, 127, 255, 0)
        temp_im, conts, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        dots = [cv2.boundingRect(c) for c in conts]
        dots = [d for d in dots if d[2] * d[3] < self.dot_size]

        return len(dots)
