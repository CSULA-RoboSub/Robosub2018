from modules.sensors.computer_vision import DiceDetector
from task import Task
from modules.controller.cv_controller import CVController
from modules.sensors.imu.gather_rotation import GetRotation
from modules.control.navigation import Navigation
from threading import Thread, Lock
import time

from collections import Counter
from itertools import combinations

class Dice(Task):
    
    def __init__(self, Houston):
        """ To initialize Dice """
        super(Dice, self).__init__()

        ################ INSTANCES ################
        self.houston = Houston
        self.cvcontroller = CVController()
        self.detectdice = None

        ################ THRESHOLD VARIABLES ################
        #self.found_threshold = 300

        ################ FLAG VARIABLES ################
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        ################ TIMER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0

        ################ DICTIONARIES ################
        self.coordinates = []

        ################ AUV MOBILITY VARIABLES ################
        self.r_power=100
        self.h_power=100
        self.m_power=120

        ################ THREAD VARIABLES ################  
        self.thread_dice = None
        self.mutex = Lock()


    def reset(self):
        self.detectdice = None

        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        self.not_found_timer = 0
        self.found_timer = 0

        self.coordinates = []

        self.thread_dice = None

    def start(self, m_power=120, rotation=15):
        self.navigation.start()
        #self.run_detect_for_task(m_power, rotation)
    
    def stop(self):
        self.navigation.stop()

    def run_detect_for_task(self, m_power=120, rotation=15):
        self.reset_thread()

        self.thread_dice = Thread(target = self.detect, args = (m_power,rotation))
        #self.thread_dice = Thread(target=self.test)
        self.thread_dice.start()
        #self.thread_dice.join()

    def reset_thread(self):
        if self.thread_dice:
            self.thread_dice = None

    def detect(self, frame):
        print('detect_dice')
        if not self.detectdice:
            self.detectdice = DiceDetector.DiceDetector()

        #found, coordinates = self.detectdice.detect()

        return self.detectdice.detect()

    def navigate(self, navigation, found, coordinates, power, rotation):
        pass
    
    def complete(self):
        pass

    def bail_task(self):
        pass
        
    def search(self):
        pass

    def restart_task(self):
        pass
