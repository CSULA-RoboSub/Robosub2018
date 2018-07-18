import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task

# from modules.sensors.computer_vision import CashInDetector

# from cash_in_maneuver import CashInManeuver

class CashIn(Task):
    
    def __init__(self, Houston):
        """ To initialize CashIn """
        super(CashIn, self).__init__()

        self.coordinates = []
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        self.not_found_timer = 0
        self.found_timer = 0
        ################ INSTANCES ################
        self.houston = Houston
        # self.cash_in_maneuver = CashInManeuver()
        self.detectcashin = None

        ################ THRESHOLD VARIABLES ################

        ################ FLAG VARIABLES ################
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
        self.stop_task = False
        self.is_task_running = False
        self.is_complete = False

        ################ TIMER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0

        ################ DICTIONARIES ################

        ################ AUV MOBILITY VARIABLES ################

        ################ THREAD VARIABLES ################
        self.thread_cash_in = None
        self.mutex = Lock()
        
    def reset(self): 
        pass

    def start(self):
        pass
    
    def stop(self):
        pass
        
    def detect(self, frame):
        print('detect_dice')
        if not self.detectcashin:
            #self.detectcashin = CashInDetector.CashInDetector()
            pass

        found, gate_coordinates = self.detectcashin.detect()
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                gate_coordinates[0] = 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True
            self.task_num += 1

    def navigate(self, navigation, found, coordinates, power, rotation):
        pass
    
    def complete(self):
        pass

    def bail_task(self):
        pass

    def restart_task(self):
        pass
        
    def search(self):
        pass

    def run_detect_for_task(self):
        pass

    def reset_thread(self):
        pass

    def get_most_occur_coordinates(self): 
        pass 