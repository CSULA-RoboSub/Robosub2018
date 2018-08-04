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
        self.task_name = 'path_follow'
        
        ################ THRESHOLD VARIABLES ################
        self.path_kill_threshold = 400
        self.path_kill_direction = ''
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
        
        # self.path_phases = {
        #     None: self.path_maneuver.no_shape_found,
        #     'vertical': self.path_maneuver.vertical,
        #     'horizontal': self.path_maneuver.horizontal,
        #     'square': self.path_maneuver.horizontal
        # }

        ################ AUV MOBILITY VARIABLES ################
        self.r_power = 100
        self.h_power = 100
        self.m_power = 120

        ################ THREAD VARIABLES ################
        self.thread_path = None
        self.mutex = Lock()

        ################ PATH VARIABLES ################

        ################ PATH CONSTANTS ################
        self.frame_height_max = 480
        self.frame_width_max = 744
        self.frame_area = self.frame_width_max * self.frame_height_max
        self.close_enough_area = 56000
        self.area_ratio = 0.50

        ################ ROI VARIABLES ################

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
        # task_name = 'path_follow' 
        cvcontroller.start(task_name)
        count = 0
        self.mutex.acquire()
        self.path_maneuver.is_running_task = True
        time.sleep(1)
        while not self.stop_task and not self.complete() and count <= self.path_kill_threshold:
            navigation.do_depth_cap(self.h_power)

            found, directions, shape, width_height_ratio = cvcontroller.detect(task_name)
            if found:
                self.direction_list.append(directions)

            if (time.time()-self.last_time > 0.05):
                self.last_time = time.time()
                count += 1

                try:
                    most_occur_coords = self.get_most_occur_coordinates(self.direction_list, self.counter)
                except:
                    most_occur_coords = [0, 0, 0]

                self.navigate(navigation, found, most_occur_coords, m_power, rotation, shape, width_height_ratio)

                self.counter = Counter()
                self.direction_list = []

            #     # print 'running {} task'.format(task_name)
            #     # print 'widthxheight: {}'.format(width_height_ratio)
            #     # print 'current count: {}'.format(count)
            #     # print 'coordinates: {}'.format(most_occur_coords)
            #     # print '--------------------------------------------'
            #     # print 'type: navigation cv 0, or task to cancel task'

        cvcontroller.stop()
        navigation.cancel_all_nav()
        # navigation.cancel_and_r_nav(self.path_kill_direction, 10, 90)
        navigation.go_to_depth(9, self.h_power)
        time.sleep(3)

        #save waypoint for dice finish
        navigation.clear_waypoints()
        navigation.enqueue_current_waypoint()
        navigation.saved_heading_path1 = navigation.waypoint.heading

        navigation.m_nav('distance', 'forward', self.m_power, 9.5)

        # ########## last ditch ################################### 
#         #save waypoint for dice finish

#         navigation.m_nav('distance', 'forward', self.m_power, self.m_distance_forward_ram_dice)
        # navigation.r_nav('left', 10, 90)

#         time.sleep(10)
# ###############################################
#         navigation.m_nav('distance', 'backward', self.m_power, self.m_distance_backward_ram_dice)

#         time.sleep(10)
# ###############################################
#         self.is_running_dice2_forward = True
#         navigation.m_nav('distance', 'forward', self.m_power, self.m_distance_forward_ram_dice)
#         navigation.r_nav('right', 8, 90)

#         time.sleep(10)
# ###############################################
#         self.is_running_dice2_backward = True
#         navigation.m_nav('distance', 'backward', self.m_power, self.m_distance_backward_ram_dice)

#         time.sleep(10)
# ###############################################

        self.path_maneuver.is_running_task = False
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
        print 'detect path'

    # navigate ##################################################################################

    def navigate(self, navigation, found, coordinates, power, rotation, shape, width_height_ratio):
        # print 'navigate path'
        # self.path_phases[shape](navigation, coordinates, power, rotation, width_height_ratio)
        if found:
            if not self.path_maneuver.is_initial_centered and coordinates[0] == 0 and coordinates[1] == 0:
                # print('self.path_maneuver.is_initial_centered = True')
                self.path_maneuver.is_initial_centered = True

            elif not self.path_maneuver.is_close_enough:
                if width_height_ratio[0] * width_height_ratio[1] >= self.close_enough_area and width_height_ratio[2] > self.area_ratio:
                    #once we're close enough to the path 
                    # print('self.path_maneuver.is_close_enough = True')
                    self.path_maneuver.is_close_enough = True
                    navigation.cancel_h_nav(self.path_maneuver.h_power)

            elif self.path_maneuver.is_close_enough and (not self.path_maneuver.is_frame_height_max) and not self.path_maneuver.is_no_longer_frame_height_max and (float(width_height_ratio[1]) >= (float(self.frame_height_max) * 0.95)):
                #trigger when we first fill camera height with path
                # print('self.path_maneuver.is_frame_height_max = True')
                self.path_maneuver.is_frame_height_max = True

            elif self.path_maneuver.is_close_enough and ((self.path_maneuver.is_frame_height_max and not self.path_maneuver.is_no_longer_frame_height_max and (float(width_height_ratio[1]) <= (float(self.frame_height_max) * 0.38))) or not found):
                #trigger once we've filled camera with path and are now starting to leave the path
                # print('self.path_maneuver.is_no_longer_frame_height_max = True')
                self.path_maneuver.is_no_longer_frame_height_max = True

        ########################################################################################
        if found:
            if not self.path_maneuver.is_close_enough and not self.path_maneuver.is_initial_centered and coordinates[0] == 0 and coordinates[1] == 0:
                #dive toward bottom of path roi
                navigation.cancel_m_nav(self.path_maneuver.m_power_strafe)
                # self.path_maneuver.dive_to_path(navigation)

            # elif coordinates[0] == 0 and coordinates[1] == 0 and self.path_maneuver.is_close_enough:
            elif not self.path_maneuver.is_close_enough and self.path_maneuver.is_initial_centered and coordinates[0] == 0 and coordinates[1] == 0:
                self.path_maneuver.dive_to_path(navigation)
                navigation.cancel_m_nav(self.path_maneuver.m_power_strafe)

            elif not self.path_maneuver.is_close_enough:
                #center sub on path right axis
                if coordinates[0] != 0:
                    self.path_maneuver.center_x_or_move_forward(navigation, coordinates[0])
                #center sub on path forward axis
                elif coordinates[1] != 0:
                    self.path_maneuver.center_y(navigation, coordinates[1])

            #rotation now turns on at this point
            elif self.path_maneuver.is_close_enough and not self.path_maneuver.is_no_longer_frame_height_max:
                #rotate and center right axis/move forward
                self.path_maneuver.follow_path(navigation, coordinates[0], coordinates[2])

            else:
                navigation.cancel_all_nav()
        #finished/not found/error state
        else:
            navigation.cancel_all_nav()

    # complete ##################################################################################
    def complete(self):
        # self.is_complete = self.path_maneuver.completed_path_check()
        return self.path_maneuver.completed_path_check()

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
    def get_most_occur_coordinates(self, direction_list, counter):
        for sublist in direction_list:
            counter.update(combinations(sublist, 3))
        for key, count in counter.most_common(1):
            most_occur = key
        return most_occur

    def is_running_task(self):
        return self.path_maneuver.is_running_task

