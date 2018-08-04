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
        self.task_name = 'roulette'
        ################ INSTANCES ################
        self.houston = Houston
        self.roulette_maneuver = RouletteManeuver()

        ################ THRESHOLD VARIABLES ################
        self.not_found_threshold = 200
        self.found_threshold = 200
        self.cant_find_threshold = 2000
        self.centered_on_correct_color_threshold = 20

        ################ FLAG VARIABLES ################
        self.is_found = False
        self.is_done = False
        self.stop_task = False
        self.is_complete = False
        self.is_ball_dropped = False

        ################ TIMER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        self.counter = Counter()
        self.cant_find_counter = 0
        self.centered_on_correct_color_counter = 0

        ################ DICTIONARIES ################
        self.direction_list = []
        self.drop_color_options = {
            0: 'black',
            1: 'red',
            2: 'green'
        }

        self.roulette_vertical_m_nav = {
            -1: 'backward',
             0: 'staying',
             1: 'forward'
        }

        self.roulette_horizontal_m_nav = {
            -1: 'left',
             0: 'staying',
             1: 'right'
        }

        # self.drop_numbers?
        ################ CONSTANTS ###########################
        self.roulette_camera_direction = 'down'

        ################ AUV MOBILITY VARIABLES ################
        self.r_power=100
        self.h_power=100
        self.m_power=120
        self.depth_change = -1

        ################ THREAD VARIABLES ################
        self.mutex = Lock()

        ################ ROULETTE VARIABLES ################
        self.drop_color = 'black'
        # self.drop_color = 'red'
        # self.drop_color = 'green'

        ################ FRAME CONSTANTS ################
        self.frame_height = 480
        self.frame_width = 744
        self.frame_area = self.frame_width*self.frame_height

        ################ ROI VARIABLES ################
        self.roi_height = 0
        self.roi_width = 0
        self.roi_area = 0

        self.reset()
    # reset ##################################################################################
    def reset(self):
        self.is_found = False
        self.is_done = False
        self.stop_task = False
        self.is_complete = False
        self.is_ball_dropped = False

        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        self.counter = Counter()
        self.centered_on_correct_color_counter = 0

        self.roi_height = 0
        self.roi_width = 0
        self.roi_area = 0

    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        self.local_cvcontroller = cvcontroller
        cvcontroller.start(task_name)
        cvcontroller.change_camera_to(self.roulette_camera_direction, task_name)
        count = 0
        self.mutex.acquire()
        while not self.stop_task and not self.complete():
            # try:
            navigation.do_depth_cap(self.h_power)
            found, direction, shape, width_height = cvcontroller.detect(task_name)
            self.roi_area = width_height[0]*width_height[1]

            if found:
                self.direction_list.append(direction)

            if (time.time()-self.last_time > 0.05):
                count += 1
                
                try:
                    most_occur_coords = self.get_most_occur_coordinates(self.direction_list, self.counter)
                except:
                    most_occur_coords = [0, 0]

                # print 'running {} task'.format(task_name)
                # print 'roi area: {}'.format(self.roi_area)
                # print 'current count: {}'.format(count)
                # print 'coordinates: {}'.format(most_occur_coords)
                # print '--------------------------------------------'
                # print 'type: navigation cv 0, or task to cancel task'
                self.navigate(navigation, found, most_occur_coords, m_power, rotation, shape, self.roi_area)
                
                self.last_time = time.time()
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
    def navigate(self, navigation, found, coordinates, power, rotation, shape, roi_area):
        # if not self.roulette_maneuver.is_moving_forward:
        #     navigation.cancel_r_nav()
        #     navigation.cancel_m_nav()
        #     navigation.cancel_h_nav()
        
        # if found:
        #     if roi_area > (self.frame_area/2):
        #         navigation.cancel_h_nav()
        #     else:
        #         navigation.h_nav('down', self.depth_change, self.h_power)
        #     #TODO must ensure there will be no more height adjustment when it is auv is close enough
        #     self.roulette_maneuver.go_over_black(navigation, found, coordinates, m_power, rotation, shape)

        if not self.roulette_maneuver.is_centered and found:
            if coordinates[0] == 0 and coordinates[1] == 0:
                self.centered_on_correct_color_counter += 1
                
            elif coordinates[0] != 0 or coordinates [1] != 0:
                self.centered_on_correct_color_counter = 0

            if coordinates[0] == 0 and coordinates [1] == 0 and self.centered_on_correct_color_counter >= self.centered_on_correct_color_threshold:
                self.roulette_maneuver.is_centered = True
                # TODO lower height of sub to get closer to roulette than drop the ball
            else:
                self.roulette_maneuver.center_color(navigation, coordinates, power)
        else:
            pass
            # must utilize pingers here to search for the roulette table in specific direction

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