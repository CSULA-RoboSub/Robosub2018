'''
Dice task will handle the actual all the individual compenents
which need to be completed to handle the task including which dice need to be searched for
'''
import DiceDetector as detector
import Task


class DiceTask:

    def __init__(self):
        self.dice_dict = {1: False, 2: False, 3: False, 4: False, 5: False, 6: False}
        self.found = False
        self.complete = False
    '''
    def run_task(self):
        while not self.found:
            found = detector.locate_dice()
    '''
    def is_complete(self):
        return self.complete
