from modules.sensors.computer_vision import DiceDetector
from task import Task
from dice_maneuver import DiceManeuver
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
        self.dice_maneuver = DiceManeuver()
        self.detectdice = None

        ################ THRESHOLD VARIABLES ################
        #self.found_threshold = 300

        ################ FLAG VARIABLES ################
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
        self.stop_task = False
        self.is_task_running = False

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

        self.dice_maneuver.reset()
    
    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        self.local_cvcontroller = cvcontroller
        self.is_task_running = True
        cvcontroller.start(task_name)
        self.mutex.acquire()
        while not self.stop_task:
            #TODO
            #cv controller and detect go here
            # try:
            found, direction, shape, width_height = cvcontroller.detect(task_name)
                #self.navigate(navigation, found, coordinates, m_power, rotation)
            # except:
            #     print 'dice detect error'

        cvcontroller.stop()
        self.mutex.release()
        self.is_task_running = False
    
    # stop ##################################################################################
    def stop(self):
        self.local_cvcontroller.stop

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
        print 'must navigate to here'
    
    # complete ##################################################################################
    def complete(self):
        pass

    # bail_task ##################################################################################
    def bail_task(self):
        pass
        
    def search(self):
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
