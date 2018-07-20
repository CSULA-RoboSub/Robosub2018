import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task

from path_maneuver import PathManeuver

class Path(Task):
    
    def __init__(self, Houston):
        """ To initialize Path """
        super(Path, self).__init__()

        ################ INSTANCES ################
        self.houston = Houston
        self.path_maneuver = PathManeuver()
        self.detectpath = None
        
        ################ THRESHOLD VARIABLES ################

        ################ FLAG VARIABLES ################
        self.is_found = False
        self.stop_task = False
        self.is_complete = False
        self.is_camera_changed = False

        ################ TIMER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        self.counter = Counter()

        ################ DICTIONARIES ################
        self.direction_list = []
        
        self.path_phases = {
            None: self.path_maneuver.no_shape_found,
            'vertical': self.path_maneuver.vertical,
            'horizontal': self.path_maneuver.horizontal,
            'square': self.path_maneuver.horizontal
        }

        ################ AUV MOBILITY VARIABLES ################
        self.r_power=100
        self.h_power=100
        self.m_power=120

        ################ THREAD VARIABLES ################
        self.thread_path = None
        self.mutex = Lock()

        ################ PATH VARIABLES ################

    
    # reset ##################################################################################
    def reset(self): 
        self.detectpath = None

        self.is_found = False
        self.is_complete = False
        self.is_camera_changed = False

        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        self.counter = Counter()

        self.direction_list = []

        self.thread_path = None
        self.stop_task = False

        self.path_maneuver.reset()
    
    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        self.local_cvcontroller = cvcontroller
        cvcontroller.start(task_name)
        count = 0
        self.mutex.acquire()
        while not self.stop_task and not self.complete():
            # try:
            # found, direction, shape, width_height = cvcontroller.detect(task_name)
            found = None
            direction = None
            shape = None
            width_height = None
            # TODO may be removed. only added to ensure methods are working

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

            if self.path_maneuver.is_no_more_path:
                self.is_camera_changed = True
                cvcontroller.change_camera_to('forward', 'dice')
            
            self.counter = Counter()
            self.direction_list = []
            # except:
            #     print 'path detect error'
        cvcontroller.stop()
        self.mutex.release()
    
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
    def navigate(self, navigation, found, coordinates, power, rotation, shape, width_height):
        print 'navigate path'

        if not self.path_maneuver.is_moving_forward:
            navigation.cancel_r_nav()
            navigation.cancel_m_nav()
            navigation.cancel_h_nav()

        self.path_phases[shape](navigation, coordinates, power, rotation, width_height)
    
    # complete ##################################################################################
    def complete(self):
        self.is_complete = self.path_maneuver.completed_path_check()
        return self.is_complete

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
