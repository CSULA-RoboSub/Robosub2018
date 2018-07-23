import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task

from dice_maneuver import DiceManeuver


class Dice(Task):
    
    def __init__(self, Houston):
        """ To initialize Dice """
        super(Dice, self).__init__()

        ################ INSTANCES ################
        self.houston = Houston
        self.dice_maneuver = DiceManeuver()

        ################ THRESHOLD VARIABLES ################
        #self.found_threshold = 300
        self.back_up_threshold = 200

        ################ FLAG VARIABLES ################
        self.is_found = False
        self.stop_task = False
        self.is_complete = False
        self.is_die_number_changed = False

        ################ TIMER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        # self.back_up_counter = 0
        self.counter = Counter()

        ################ DICTIONARIES ################
        self.direction_list = []
        self.dies = {
            0: 1,
            1: 6
        }

        self.die_phases = {
            None: self.dice_maneuver.no_shape_found,
            'vertical': self.dice_maneuver.centered_and_shape_found,
            'horizontal': self.dice_maneuver.centered_and_shape_found,
            'square': self.dice_maneuver.centered_and_shape_found
        }

        ################ AUV MOBILITY VARIABLES ################
        self.r_power=100
        self.h_power=100
        self.m_power=120

        ################ THREAD VARIABLES ################  
        self.thread_dice = None
        self.mutex = Lock()

        ################ DICE VARIABLES ################
        self.die_1 = 1
        self.die_2 = 6

        ################ FRAME VARIABLES ################
        self.laptop_camera = (640, 480)
        self.sub_driver_camera = (744, 480)

    # reset ################################################################################## 
    def reset(self):
        self.is_found = False
        self.is_done = False
        self.is_complete = False
        self.is_die_number_changed = False

        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        self.counter = Counter()

        self.direction_list = []

        self.thread_dice = None
        self.stop_task = False
        
        self.dice_maneuver.reset()
    
    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        self.local_cvcontroller = cvcontroller
        cvcontroller.camera_direction = 'forward'
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
            
            if self.dice_maneuver.is_1st_die_touched and not self.is_die_number_changed:
                self.is_die_number_changed = True
                cvcontroller.change_die_num()
                self.dice_maneuver.reset_after_1st_die()
            # except:
            #     print 'dice detect error'

        cvcontroller.stop()
        self.mutex.release()
    
    # stop ##################################################################################
    def stop(self):
        self.local_cvcontroller.stop()

    # run_detect_for_task ##################################################################################
    def run_detect_for_task(self, m_power=120, rotation=15):
        pass

    # reset_thread ##################################################################################
    def reset_thread(self):
        pass

    # detect ##################################################################################
    def detect(self, frame):
        pass
        
    # navigate ##################################################################################
    def navigate(self, navigation, found, coordinates, power, rotation, shape, width_height):
        if not self.dice_maneuver.is_moving_forward:
            navigation.cancel_r_nav()
            navigation.cancel_m_nav()
            navigation.cancel_h_nav()

        # if found:
        if not self.dice_maneuver.is_rotated_to_center and shape:
            if coordinates[0] == 0:
                self.dice_maneuver.is_rotated_to_center = True
            else:
                self.dice_maneuver.rotate_to_center(navigation, coordinates, power, rotation)
        else:
            self.die_phases[shape](navigation, coordinates, power, rotation, width_height)    
    
    # complete ##################################################################################
    def complete(self):
        self.is_complete = self.dice_maneuver.completed_dice_check()
        return self.is_complete

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
    def get_most_occur_coordinates(self, direction_list, counter): 
        for sublist in direction_list:
            counter.update(combinations(sublist, 2))
        for key, count in counter.most_common(1):
            most_occur = key
        return most_occur 
