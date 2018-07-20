import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task

from roulette_maneuver import RouletteManeuver
class Roulette(Task):
    
    def __init__(self, Houston):
        """ To initialize Roulette """
        super(Roulette, self).__init__()

        ################ INSTANCES ################
        self.houston = Houston
        self.roulette_maneuver = RouletteManeuver()

        ################ THRESHOLD VARIABLES ################
        self.not_found_threshold = 200
        self.found_threshold = 200

        ################ FLAG VARIABLES ################
        self.is_found = False
        self.is_done = False
        self.stop_task = False
        self.is_complete = False
        self.is_coin_dropped = False

        ################ TIMER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        self.counter = Counter()

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
        self.is_found = False
        self.is_done = False
        self.stop_task = False
        self.is_complete = False
        self.is_coin_dropped = False

        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        self.counter = Counter()

    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        self.local_cvcontroller = cvcontroller
        cvcontroller.start(task_name)
        count = 0
        self.mutex.acquire()
        while not self.stop_task and not self.complete():
            # try:
            found, direction, shape, width_height = cvcontroller.detect(task_name)
            if found:
                self.direction_list.append(direction)

            if (time.time()-self.last_time > 0.05):
                self.last_time = time.time()
                count += 1
                
                try:
                    most_occur_coords = self.get_most_occur_coordinates(self.direction_list, self.counter)
                except:
                    most_occur_coords = [0, 0]

                print 'running {} task'.format(task_name)
                print 'widthxheight: {}'.format(width_height)
                print 'current count: {}'.format(count)
                print 'coordinates: {}'.format(most_occur_coords)
                print '--------------------------------------------'
                print 'type: navigation cv 0, or task to cancel task'
                self.navigate(navigation, found, most_occur_coords, m_power, rotation, shape, width_height)
                
                self.counter = Counter()
                self.direction_list = []
            # except:
            #     print 'roulette detect error'

        cvcontroller.stop()
        self.mutex.release()
    
    # stop ##################################################################################
    def stop(self):
        pass

    # detect ##################################################################################
    def detect(self, frame):
        pass
    
    # navigate ##################################################################################
    def navigate(self, navigation, found, most_occur_coords, m_power, rotation, shape, width_height):
        self.roulette_maneuver.go_over_black()
    
    # complete ##################################################################################
    def complete(self):
        self.is_complete = self.roulette_maneuver.completed_roulette_check()
        return self.is_complete

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