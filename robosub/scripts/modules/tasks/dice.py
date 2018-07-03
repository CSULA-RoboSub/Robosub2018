
from modules.sensors.computer_vision import DiceDetector as dd
from modules.sensors.computer_vision import DicePlatformDetector as dpd

from task import Task

class Dice(Task):
    
    def __init__(self, Houston, navi):
        """ To initialize Dice """
        super(Dice, self).__init__()

        self.houston = Houston
        self.dice_detector = dd.DiceDetector()
        self.platform_detector = dpd.DicePlatformDetector()
        self.coordinates = []
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
        self.navigation = navi
        self.not_found_timer = 0
        self.found_timer = 0
        self.task_complete = False
        self.at_platform = False
        self.sum = 7
        self.dice_to_find = []

    def detect(self, frame):
        found, coordinates = self.dice_detector.detect()
        return found, coordinates

    def find_platform(self, frame):
        found, coords = self.platform_detector.detect(frame)
        dice_found, _ = self.detect(frame)
        return found, coords, dice_found

    def center(self,frame, x_die, y_die):
        width, height, _ = frame.shape()
        center = [width / 2 , height / 2]
        x_buffer = center[0] / 7
        y_buffer = center[1] / 7

        '''
            FIND THE CODE TO STRAFE 
        '''
        if x_die < center[0] - x_buffer:
            self.coordinates[0] = -1
        elif x_die > center[0] + x_buffer:
            self.coordinates[0] = 1
        else:
            self.coordinates[0] = 0

        if y_die < center[0] - y_buffer:
            self.coordinates[0] = -1
        elif y_die > center[0] + y_buffer:
            self.coordinates[0] = 1
        else:
            self.coordinates[0] = 0

        if self.coordinates == [0,0]:
            return True
        else:
            #Command the sub to strafe in the directions found
            return False

    def navigate(self, navigation, found, coordinates, power, rotation):
        pass
    
    def complete(self):
        pass

    def bail_task(self):
        pass

    def restart_task(self):
        pass

    def start(self):
        self.navigation.start()
    
    def stop(self):
        self.navigation.stop()