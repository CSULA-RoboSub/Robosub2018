import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task

from slots_maneuver import SlotsManeuver

class Slots(Task):
    
    def __init__(self, Houston):
        """ To initialize Slots """
        super(Slots, self).__init__()

        self.detectslots = None
        self.coordinates = []
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
        
        self.not_found_timer = 0
        self.found_timer = 0

        ################ INSTANCES ################
        self.houston = Houston
        self.slot_maneuver = SlotsManeuver()
        self.detectslots = None

        ################ THRESHOLD VARIABLES ################
        self.phase_threshold = 100
        # self.heading_verify_threshold = 100
        self.under_threshold = 100
        self.cant_find_threshold = 2000
        self.rotated_to_center_verify_threshold = 20 
        
        ################ FLAG VARIABLES ################
        self.is_found = False
        self.stop_task = False
        self.is_complete = False

        ################ TIMER/COUNTER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0
        self.forward_counter = 0
        self.heading_verify_count = 0
        self.last_time = 0
        self.counter = Counter()
        self.rotated_to_center_verify = 0 

        ################ DICTIONARIES ###########################        
        self.movement_to_square = {
            'vertical': 'right',
            'horizontal': 'backward'
        }
                                
        # self.slots_phases = {
        #     None: self.slots_maneuver.no_shape_found,
        #     'vertical': self.slots_maneuver.vertical,
        #     'horizontal': self.slots_maneuver.horizontal,
        #     'square': self.slots_maneuver.square
        # }

        ################ CONSTANTS ###########################  

        ################ AUV MOBILITY VARIABLES ################
        self.depth_change = 1
        self.rotation_angle = 5
        self.is_heading_correct = False

        self.direction_list = []
        self.r_power=70
        self.h_power=100
        self.m_power=120   
        self.found = False
        # self.orientation_heading = None
        
        ################ THREAD VARIABLES ################    
        self.thread_slots = None
        self.mutex = Lock()

    # detect ##################################################################################
    def detect(self, frame):
        print('detect_dice')
        if not self.detectslots:
            #self.detectslots = SlotsDetector.SlotsDetector()
            pass

        found, gate_coordinates = self.detectslots.detect()
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                gate_coordinates[0] = 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True
            self.task_num += 1

    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        self.local_cvcontroller = cvcontroller
        cvcontroller.camera_direction = 'forward'
        cvcontroller.start(task_name)
        self.mutex.acquire()
        count = 0
        self.last_time = time.time()
        self.slot_maneuver.torpedo('prime', 'right')
        while not self.stop_task and not self.complete():
            print 'looping slots for testing'

            # found, directions, shape, width_height = cvcontroller.detect(task_name)
            # TODO implement slots detector for CV controller
            # if found:
                # self.direction_list.append(directions)
                # if (time.time()-self.last_time > 0.05):
                #     count += 1

                #     try:
                #         most_occur_coords = self.get_most_occur_coordinates(self.direction_list, self.counter)
                #     except:
                #         most_occur_coords = [0, 0]

                    # print 'running {} task'.format(task_name)
                    # print 'gate shape: {}, widthxheight: {}'.format(gate_shape, width_height)
                    # print 'current count: {}'.format(count)
                    # print 'coordinates: {}'.format(most_occur_coords)
                    # print '--------------------------------------------'
                    # print 'type: navigation cv 0, or task to cancel task'
                    # self.navigate(navigation, found, most_occur_coords, m_power, rotation, gate_shape, width_height)

        self.mutex.release()
        cvcontroller.stop()

    # stop ##################################################################################
    def stop(self):
        pass

    # navigation ##################################################################################
    def navigate(self, navigation, found, coordinates, power, rotation):
        pass

    # complete ##################################################################################
    def complete(self):
        return False

    # bail_task ##################################################################################
    def bail_task(self):
        pass

    # restart_task ##################################################################################
    def restart_task(self):
        pass

    #search ##################################################################################
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

    # reset ##################################################################################
    def reset(self):
        pass