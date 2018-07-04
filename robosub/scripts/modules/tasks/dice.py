from modules.sensors.computer_vision import DiceDetector
from task import Task
from modules.controller.cv_controller import CVController
from modules.sensors.imu.gather_rotation import GetRotation
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

    # reset ################################################################################## 
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
    
    # start ##################################################################################
    def start(self, m_power=120, rotation=15):
        #self.navigation.start()
        #self.run_detect_for_task(m_power, rotation)
        pass
    
    # stop ##################################################################################
    def stop(self):
        #self.navigation.stop()
        pass

    # run_detect_for_task ##################################################################################
    def run_detect_for_task(self, m_power=120, rotation=15):
        self.reset_thread()

        self.thread_dice = Thread(target = self.detect, args = (m_power,rotation))
        #self.thread_dice = Thread(target=self.test)
        self.thread_dice.start()
        #self.thread_dice.join()

    # reset_thread ##################################################################################
    def reset_thread(self):
        if self.thread_dice:
            self.thread_dice = None

    # detect ##################################################################################
    def detect(self, frame):
        print('detect_dice')
        if not self.detectdice:
            self.detectdice = DiceDetector.DiceDetector()

        #found, coordinates = self.detectdice.detect()

        return self.detectdice.detect()
    # navigate ##################################################################################
    def navigate(self, navigation, found, coordinates, power, rotation):
        pass
    
    # complete ##################################################################################
    def complete(self):
        pass

    # bail_task ##################################################################################
    def bail_task(self):
        pass

    # restart_task ##################################################################################
    def restart_task(self):
        pass
    
    # run_detect_for_task ##################################################################################
    def run_detect_for_task(self):
        pass

    # reset_thread ##################################################################################
    def reset_thread(self):
        pass

    # get_most_occur_coordinates ##################################################################################
    def get_most_occur_coordinates(self): 
        pass 