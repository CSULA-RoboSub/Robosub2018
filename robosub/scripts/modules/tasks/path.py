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
        self.r_power=50
        self.h_power=100
        self.m_power=60

        ################ THREAD VARIABLES ################
        self.thread_path = None
        self.mutex = Lock()

        ################ PATH VARIABLES ################

        ################ PATH CONSTANTS ################
        self.frame_height_max = 480
        self.frame_width_max = 744
        self.frame_area = self.frame_width_max * self.frame_height_max

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
        cvcontroller.camera_direction = 'down'
        task_name = 'path_follow' 
        cvcontroller.start(task_name)
        count = 0
        self.mutex.acquire()
        while not self.stop_task and not self.complete():
            # try:
            # found, direction, shape, width_height = cvcontroller.detect(task_name)
            # found = None
            # direction = None
            # shape = None
            # width_height = None
            # TODO may be removed. only added to ensure methods are working

            found, directions, gate_shape, width_height = cvcontroller.detect(task_name)
            if found:
                self.direction_list.append(directions)

            if (time.time()-self.last_time > 0.05):
                self.last_time = time.time()
                count += 1

                try:
                    most_occur_coords = self.get_most_occur_coordinates(self.direction_list, self.counter)
                except:
                    most_occur_coords = [0, 0, 0]

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
    # rotate always on
    # first dive til area large enough while aiming for bottom of roi
    # move forward if strafe centered
    # if not centered strafe to center

    def navigate(self, navigation, found, coordinates, power, rotation, shape, width_height):
        # print 'navigate path'
        # self.path_phases[shape](navigation, coordinates, power, rotation, width_height)
    
        if found and coordinates[0] != 0 and not self.path_maneuver.is_close_enough:
            #center sub on path right axis

        elif found and coordinates[1] != 0 and not self.path_maneuver.is_close_enough:
            #center sub on path forward axis

        elif found and coordinates[0] == 0 and coordinates[1] == 0 and not self.path_maneuver.is_close_enough:
            #dive toward bottom of path roi

        elif self.path_maneuver.is_close_enough and width_height[1] >= self.frame_height_max and not self.path_maneuver.is_frame_height_max:
        #rotation now turns on at this point
        elif self.path_maneuver.is_close_enough:
            #rotate and center right axis/move forward
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
