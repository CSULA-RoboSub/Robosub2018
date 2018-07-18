import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task
# from modules.sensors.computer_vision import RouletteDetector

# from roulette_maneuver import RouletteManeuver
class Roulette(Task):
    
    def __init__(self, Houston):
        """ To initialize Roulette """
        super(Roulette, self).__init__()

        ################ INSTANCES ################
        self.houston = Houston
        # self.roulette_maneuver = RouletteManeuver()

        ################ THRESHOLD VARIABLES ################
        self.not_found_threshold = 200
        self.found_threshold = 200

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
        self.direction_list = []
        self.drop_color_options = {
            0: 'black',
            1: 'red',
            2: 'green'
        }

        # self.drop_numbers?

        ################ AUV MOBILITY VARIABLES ################
        self.r_power=100
        self.h_power=100
        self.m_power=120

        ################ THREAD VARIABLES ################
        self.mutex = Lock()

        ################ ROULETTE VARIABLES ################
        self.drop_color = 'black'
        # self.drop_color = 'red'
        # self.drop_color = 'green'

    # reset ##################################################################################
    def reset(self):
        self.not_found_timer = 0
        self.found_timer = 0

    # start ##################################################################################
    def start(self):
        pass
    
    # stop ##################################################################################
    def stop(self):
        pass

    # detect ##################################################################################
    def detect(self, frame):
        pass
    
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

    # search ##################################################################################
    def search(self):
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