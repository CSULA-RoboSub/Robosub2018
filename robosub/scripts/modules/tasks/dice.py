from modules.sensors.computer_vision import DiceDetector
from task import Task

class Dice(Task):
    
    def __init__(self):
        """ To initialize Dice """
        super(Dice, self).__init__()

        self.detectdice = None
        self.coordinates = []
        self.is_dice_found = False
        self.is_dice_done = False

        self.not_found_timer = 0
        self.found_timer = 0

        self.is_complete_first_die = False
        self.is_complete_second_die = False

    def detect(self):
        print('detect_dice')
        if not self.detectdice:
            #self.detectdice = DiceDetector.DiceDetector()
            pass

        found, gate_coordinates = self.detectdice.detect()
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                gate_coordinates[0] = 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True
            self.task_num += 1

    def navigate(self):
        pass
    
    def complete(self):
        pass