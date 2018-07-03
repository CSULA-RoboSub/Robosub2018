
from modules.sensors.computer_vision import DiceDetector as dd
from modules.sensors.computer_vision import DicePlatformDetector as dpd

from task import Task

class Dice(Task):
    
    def __init__(self, Houston, navi):
        """ To initialize Dice """
        super(Dice, self).__init__()

        self.houston = Houston
        self.dice_detector = DiceDetector.DiceDetector()
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
        self.is_complete_first_die = False
        self.is_complete_second_die = False
        self.sum = 7
        self.dice_to_find = []

    def detect(self, frame):
        found, coordinates = self.dice_detector.detect()
        return found, coordinates

    def find_platform(self,frame):
        found, coords = self.platform_detector.detect(frame)
        return found, coords

    def find_pair(self,frame):
        die1, die2 = self.dice_detector.get_pair(frame)


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