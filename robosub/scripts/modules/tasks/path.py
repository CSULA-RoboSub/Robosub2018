import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task
# from modules.sensors.computer_vision import PathDetector

# from path_maneuver import PathManeuver

class Path(Task):
    
    def __init__(self, Houston):
        """ To initialize Path """
        super(Path, self).__init__()

        ################ INSTANCES ################
        self.houston = Houston
        # self.path_maneuver = PathManeuver()
        self.detectpath = None
        
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
        self.coordinates = []

        ################ AUV MOBILITY VARIABLES ################

        ################ THREAD VARIABLES ################
        self.thread_path = None
        self.mutex = Lock()
    
    # reset ##################################################################################
    def reset(self): 
        self.detectpath = None

        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
        self.is_complete = False

        self.not_found_timer = 0
        self.found_timer = 0

        self.coordinates = []

        self.thread_path = None

        # self.path_maneuver.reset()
    
    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        self.local_cvcontroller = cvcontroller
        self.is_task_running = True
        cvcontroller.start(task_name)
        self.mutex.acquire()
        # TODO implement path cvcontroller and path navigate
        # while not self.stop_task:
        #     try:
        #         found, direction, shape, width_height = cvcontroller.detect(task_name)
        #     except:
        #         print 'path detect error'
        
        #     self.navigate(navigation, found, coordinates, m_power, rotation)

        cvcontroller.stop()
        self.mutex.release()
        self.is_task_running = False
    
    # stop ##################################################################################
    def stop(self):
        self.local_cvcontroller.stop()
        
    # TODO will eventually remove since Houston creates thread
    # run_detect_for_task ##################################################################################
    def run_detect_for_task(self):
        pass

    # reset_thread ##################################################################################
    def reset_thread(self):
        pass

    # detect ##################################################################################
    def detect(self, frame):

        '''if not self.detectpath:
            #self.detectpath = PathDetector.PathDetector()
            pass

        found, gate_coordinates = self.detectpath.detect()
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                gate_coordinates[0] = 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True
            self.task_num += 1'''
        print 'detect path'
        self.navigate()
        self.complete()
        self.bail_task()
        self.restart_task()

        return False, [0,0]

    # navigate ##################################################################################
    def navigate(self, navigation, found, coordinates, power, rotation):
        print 'navigate path'
    
    # complete ##################################################################################
    def complete(self):
        print 'complete path'

    # bail_task ##################################################################################
    def bail_task(self):
        print 'bail task path'

    # restart_task ##################################################################################
    def restart_task(self):
        print 'restart task path'

    # search ##################################################################################
    def search(self):
        pass

    # get_most_occur_coordinates ##################################################################################
    def get_most_occur_coordinates(self): 
        pass 
