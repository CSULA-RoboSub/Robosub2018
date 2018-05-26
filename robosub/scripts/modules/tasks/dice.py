from modules.sensors.computer_vision import DiceDetector
from task import Task

class Dice(Task):
    
    def __init__(self, Houston):
        """ To initialize Dice """
        super(Dice, self).__init__()

        self.houston = Houston

        self.detectdice = None
        self.coordinates = []
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        self.not_found_timer = 0
        self.found_timer = 0

        self.is_complete_first_die = False
        self.is_complete_second_die = False

    def detect(self, frame):
        print('detect_dice')
        if not self.detectdice:
            self.detectdice = DiceDetector.DiceDetector()

        found, coordinates = self.detectdice.detect()

        return found, coordinates

    def navigate(self):
        pass
    
    def complete(self):
        pass

    def bail_task(self):
        pass

    def restart_task(self):
        pass