import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task
from modules.sensors.computer_vision import DiceDetector

from dice_maneuver import DiceManeuver


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
        self.is_complete = False

        ################ TIMER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        self.counter = Counter()

        ################ DICTIONARIES ################
        self.direction_list = []

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
        self.is_task_running = False
        self.is_done = False
        self.is_complete = False

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
        self.is_task_running = True
        cvcontroller.start(task_name)
        count = 0
        self.mutex.acquire()
        while not self.stop_task:
            try:
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

                    print 'shape: {}, widthxheight: {}'.format(shape, width_height)
                    print 'current count: {}'.format(count)
                    print 'coordinates: {}'.format(most_occur_coords)
                    print '--------------------------------------------'
                    print 'type: navigation cv 0, or task to cancel task'
                    self.navigate(navigation, found, most_occur_coords, m_power, rotation, shape, width_height)
                    
                    self.counter = Counter()
                    self.direction_list = []
                    self.last_time = time.time()
            except:
                print 'dice detect error'

        cvcontroller.stop()
        self.mutex.release()
        self.is_task_running = False
    
    # stop ##################################################################################
    def stop(self):
        self.local_cvcontroller.stop()

    #TODO remove soon, since thread is being created in Houston
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
    def navigate(self, navigation, found, coordinates, power, rotation, shape, width_height):
        if not self.dice_maneuver.is_moving_forward:
            navigation.cancel_r_nav()
            navigation.cancel_m_nav()
            navigation.cancel_h_nav()

        # if found:
        if not self.dice_maneuver.is_rotated_to_center:
            if shape:
                if coordinates[0] == 0:
                    self.dice_maneuver.is_rotated_to_center = True
                else:
                    self.dice_maneuver.rotate_to_center(navigation, coordinates, power, rotation)
            else:
                self.dice_maneuver.find_die(navigation, power, rotation)
        else:
            self.dice_maneuver.touch_die(navigation, coordinates, power, rotation)
        # else:
        #     pass

        
    
    # complete ##################################################################################
    def complete(self):
        self.is_complete = True

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
