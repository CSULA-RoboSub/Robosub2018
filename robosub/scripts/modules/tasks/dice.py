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
        self.task_name = 'dice'
        ################ THRESHOLD VARIABLES ################
        # self.found_threshold = 300
        self.back_up_threshold = 200

        ################ FLAG VARIABLES ################
        self.is_found = False
        self.stop_task = False
        self.is_complete = False
        self.is_die_number_changed = False
        self.is_first_found = False

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
        self.r_power = 100
        self.h_power = 100
        self.m_power = 120

        ################ THREAD VARIABLES ################  
        self.thread_dice = None
        self.mutex = Lock()

        ################ DICE VARIABLES ################
        self.die_1 = 1
        self.die_2 = 6

        ################ FRAME VARIABLES ################
        self.laptop_camera = (640, 480)
        self.sub_driver_camera = (744, 480)

        self.m_distance_forward_dock = 18.5
        # self.m_distance_forward_dock = 3 
        self.m_distance_forward_path = 9.5
        # self.m_distance_forward_path = 0.7
        self.m_distance_forward_ram_dice = 2.1
        self.m_distance_backward_ram_dice = 2
        # self.m_distance_forward_ram_dice = 1.1
        # self.m_distance_backward_ram_dice = 1

        self.reset()

    # reset ##################################################################################
    def reset(self):
        self.is_found = False
        self.is_done = False
        self.is_complete = False
        self.is_die_number_changed = False
        self.is_first_found = False

        self.not_found_timer = 0
        self.found_timer = 0
        self.last_time = 0
        self.counter = Counter()

        self.direction_list = []

        self.thread_dice = None
        self.stop_task = False

        self.dice_maneuver.reset()

    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=5):
        self.local_cvcontroller = cvcontroller
        self.dice_maneuver.navigation = navigation

        cvcontroller.change_dice(6)
        cvcontroller.camera_direction = 'forward'
        cvcontroller.start(task_name)
        count = 0
        self.mutex.acquire()
        self.dice_maneuver.is_running_task = True
        time.sleep(1)
        while not self.stop_task and not self.complete():
            navigation.do_depth_cap(self.h_power)
            # try:
            found, direction, shape, width_height = cvcontroller.detect(task_name)

            if found and not self.is_first_found:
                # cancel navigation passed on from path once auv first see's a dice
                self.is_first_found = True
                navigation.cancel_all_nav()

            if found:
                self.direction_list.append(direction)

            if time.time() - self.last_time > 0.05:
                self.last_time = time.time()
                count += 1

                try:
                    most_occur_coords = self.get_most_occur_coordinates(self.direction_list, self.counter)
                except:
                    most_occur_coords = [0, 0]

                self.navigate(navigation, found, most_occur_coords, m_power, rotation, shape, width_height)

                # print 'running {} task'.format(task_name)
                # print 'widthxheight: {}'.format(width_height)
                # print 'current count: {}'.format(count)
                # print 'coordinates: {}'.format(most_occur_coords)
                # print '--------------------------------------------'
                # print 'type: navigation cv 0, or task to cancel task'

                self.counter = Counter()
                self.direction_list = []

            if self.dice_maneuver.is_1st_die_touched and not self.is_die_number_changed:
                cvcontroller.change_dice(5)
                self.is_die_number_changed = True
                self.dice_maneuver.reset_after_1st_die()
            # except:
            #     print 'dice detect error'

        cvcontroller.stop()
        navigation.run_queue_waypoints()
        time.sleep(2)
        direction, degree = navigation.waypoint.get_directions_with_heading(navigation.saved_heading_path1)
        navigation.cancel_and_r_nav(direction, degree, self.r_power)

        navigation.go_to_depth(3.5, self.h_power)
        time.sleep(17)

        navigation.m_nav('distance', 'forward', self.m_power, 12.3)
        time.sleep(20)
        # navigation.m_nav('power', 'forward', self.m_power)
        self.dice_maneuver.is_running_task = False
        self.mutex.release()

    # stop ##################################################################################
    def stop(self):
        self.local_cvcontroller.stop()

    # run_detect_for_task ##################################################################################
    def run_detect_for_task(self, m_power=120, rotation=5):
        pass

    # reset_thread ##################################################################################
    def reset_thread(self):
        pass

    # detect ##################################################################################
    def detect(self, frame):
        pass

    # navigate ##################################################################################
    def navigate(self, navigation, found, coordinates, power, rotation, shape, width_height):

        if self.dice_maneuver.is_moving_forward:
            self.dice_maneuver.nothing_found_counter = 0
            return

        # if found:
        # if not self.dice_maneuver.is_rotated_to_center and shape:
        #     if coordinates[0] == 0:
        #         self.dice_maneuver.is_rotated_to_center = True
        #     else:
        #         self.dice_maneuver.rotate_to_center(navigation, coordinates, power, rotation)
        # else:
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
