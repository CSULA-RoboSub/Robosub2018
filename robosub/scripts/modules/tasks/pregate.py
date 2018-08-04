import time
import rospy

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task
from robosub.msg import RControl
from robosub.msg import HControl
from robosub.msg import MControl

class PreGate(Task):
    
    def __init__(self, Houston):
        """ To initialize pregate """
        super(PreGate, self).__init__()
        
        self.is_do_last_resort = False

        self.task_name = 'pregate'
        self.reset()
        ###########################################
        ############ IMPORTANT VARIABLES ##########
        self.selected_heading = 'c'
        self.coin = 'tails'
        self.coin_direction = 'right'
        ################ INSTANCES ################
        self.houston = Houston
        
        ################ THRESHOLD VARIABLES ################

        ################ FLAG VARIABLES ################
        self.stop_task = False
        self.is_complete = False
        self.is_running_task_pregate = False

        self.is_busy = False
        self.is_running_rotation = False
        ################ TIMER VARIABLES ################

        ################ DICTIONARIES ################
        self.headings = {
            'a': 317.0,
            'b': 5.0,
            'c': 160.0,
            'd': 167.5
        }

        self.coin_headings = {
            'tails' : 81.5,
            'heads' : (81.5 + 90)
        }

        ################ AUV MOBILITY VARIABLES ################
        self.r_power=100
        self.h_power=100
        self.m_power=160
        self.h_depth = 5

        ################ THREAD VARIABLES ################
        self.mutex = Lock()

        ################ PREGATE VARIABLES ################

        ################ PREGATE CONSTANTS ################
        # self.m_distance_forward_dock = 15.9 
        self.m_distance_forward_dock = 3 
        # self.m_distance_forward_path = 9.3
        self.m_distance_forward_path = 0.7
        # self.m_distance_forward_ram_dice = 2.1
        # self.m_distance_backward_ram_dice = 2
        self.m_distance_forward_ram_dice = 1.1
        self.m_distance_backward_ram_dice = 1
        
        ################ PREGATE ROS VARIABLES ################
        rospy.Subscriber('rotation_control_status', RControl, self.r_status_callback, queue_size=100)
        rospy.Subscriber('height_control_status', HControl, self.h_status_callback, queue_size=100)
        rospy.Subscriber('movement_control_status', MControl, self.m_status_callback, queue_size=100)
    # reset ##################################################################################
    def reset(self): 
        self.stop_task = False
        self.is_complete = False
        self.is_running_task_pregate = False
        self.is_running_move_forward_from_dock = False
        self.is_running_path = False
        self.is_running_dice1_forward = False
        self.is_running_dice1_backward = False
        self.is_running_dice2_forward = False
        self.is_running_dice2_backward = False

        self.is_busy = False
        self.is_running_rotation = False
    
    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        # self.local_cvcontroller = cvcontroller
        # cvcontroller.camera_direction = 'forward'
        # cvcontroller.start('gate')
        # count = 0
        self.mutex.acquire()
        self.is_running_task_pregate = True
        self.is_busy = False
        self.is_running_rotation = False

        navigation.h_nav('down', 3.5, self.h_power)
        
        while not self.stop_task and not self.is_complete:
            # found, directions, gate_shape, width_height = cvcontroller.detect(task_name)

            if not self.is_busy and not self.is_running_rotation:
                # print('in rotation')
                self.is_busy = True
                self.is_running_rotation = True
                #run rotation
                if navigation.saved_heading is not None:
                    print(str(navigation.saved_heading))
                    direction, degree = navigation.waypoint.get_directions_with_heading(navigation.saved_heading)

                else:
                    # direction, degree = navigation.waypoint.get_directions_with_heading(self.headings[self.selected_heading])
                    direction = self.coin_direction
                    degree = self.coin_headings[self.coin]
                    print(str(degree))
                print('direction: {} degree: {}'.format(str(direction), str(degree)))
                #set is running rotation
                navigation.r_nav(direction, degree, self.r_power)

            elif self.is_busy and not self.is_running_rotation:
                # print('in move')
                #run forward
                #set complete
                self.is_complete = True

        # cvcontroller.stop()

        if not self.is_do_last_resort:
            navigation.m_nav('distance', 'forward', self.m_power, self.m_distance_forward_dock)
            time.sleep(4)

        ########## last ditch ###################################
        else:
            self.is_running_move_forward_from_dock = True
            navigation.m_nav('distance', 'forward', self.m_power, self.m_distance_forward_dock)

            while not self.stop_task and self.is_running_move_forward_from_dock:                
                navigation.do_depth_cap(self.h_power)

            navigation.go_to_depth(11, self.h_power)
            while not self.stop_task and not navigation.is_at_assigned_depth():
                navigation.do_depth_cap(self.h_power)

            time.sleep(1)

            self.is_running_path = True
            navigation.m_nav('distance', 'forward', self.m_power, self.m_distance_forward_path)
            navigation.r_nav('left', 45, 90)
            navigation.go_to_depth(9, self.h_power)

            while not self.stop_task and self.is_running_path:
                navigation.do_depth_cap(self.h_power)

            time.sleep(1)
            #save waypoint for dice finish
            navigation.clear_waypoints()
            navigation.enqueue_current_waypoint()
            navigation.saved_heading_path1 = navigation.waypoint.heading

            self.is_running_dice1_forward = True
            navigation.m_nav('distance', 'forward', self.m_power, self.m_distance_forward_ram_dice)
            navigation.r_nav('left', 4, 90)
            while not self.stop_task and self.is_running_dice1_forward:
                navigation.do_depth_cap(self.h_power)

            time.sleep(1)

            self.is_running_dice1_backward = True
            navigation.m_nav('distance', 'backward', self.m_power, self.m_distance_backward_ram_dice)

            while not self.stop_task and self.is_running_dice1_backward:
                navigation.do_depth_cap(self.h_power)

            time.sleep(1)

            self.is_running_dice2_forward = True
            navigation.m_nav('distance', 'forward', self.m_power, self.m_distance_forward_ram_dice)
            navigation.r_nav('right', 8, 90)

            while not self.stop_task and self.is_running_dice2_forward:
                navigation.do_depth_cap(self.h_power)

            time.sleep(1)

            self.is_running_dice2_backward = True
            navigation.m_nav('distance', 'backward', self.m_power, self.m_distance_backward_ram_dice)

            while not self.stop_task and self.is_running_dice2_backward:
                navigation.do_depth_cap(self.h_power)

            time.sleep(1)

        print 'PreGate Finish'

        self.is_running_task_pregate = False
        self.mutex.release()
    
    # stop ##################################################################################
    def stop(self):
        # self.local_cvcontroller.stop()
        pass
        
    # TODO will eventually remove since Houston creates thread
    # run_detect_for_task ##################################################################################
    def run_detect_for_task(self):
        pass

    # reset_thread ##################################################################################
    def reset_thread(self):
        pass

    # detect ##################################################################################
    def detect(self, frame):
        pass

    def navigate(self, navigation, found, coordinates, power, rotation, shape, width_height_ratio):
        # print 'navigate pregate'
        pass

    # complete ##################################################################################
    def complete(self):
        return self.is_complete

    # bail_task ##################################################################################
    def bail_task(self):
        print 'bail task pregate'

    # restart_task ##################################################################################
    def restart_task(self):
        print 'restart task pregate'

    # search ##################################################################################
    def search(self):
        pass

    # get_most_occur_coordinates ##################################################################################
    def get_most_occur_coordinates(self, direction_list, counter):
        for sublist in direction_list:
            counter.update(combinations(sublist, 2))
        for key, count in counter.most_common(1):
            most_occur = key
        return most_occur

    #rotaton control status callback
    def r_status_callback(self, rotation_status):
        # print(rotation_status)
        if not self.is_running_task_pregate:
            return

        if self.is_busy and self.is_running_rotation:
            if rotation_status.state == 1:
                #set is running rotation false
                self.is_running_rotation = False

    def h_status_callback(self, height_status):
        if not self.is_running_task_pregate:
            return

        if height_status.state == 1:
            self.is_complete = True

    def m_status_callback(self, movement_status):
        if not self.is_running_task_pregate:
            return

        if self.is_running_move_forward_from_dock and movement_status.state == 0  and abs(movement_status.distance - self.m_distance_forward_dock) < 0.001 and movement_status.power == 0:
            self.is_running_move_forward_from_dock = False

        elif self.is_running_path and movement_status.state == 0  and abs(movement_status.distance - self.m_distance_forward_path) < 0.001 and movement_status.power == 0:
            self.is_running_path = False

        elif self.is_running_dice1_forward and movement_status.state == 0  and abs(movement_status.distance - self.m_distance_forward_ram_dice) < 0.001 and movement_status.power == 0:
            self.is_running_dice1_forward = False

        elif self.is_running_dice1_backward and movement_status.state == 0  and abs(movement_status.distance - self.m_distance_backward_ram_dice) < 0.001 and movement_status.power == 0:
            self.is_running_dice1_backward = False

        elif self.is_running_dice2_forward and movement_status.state == 0  and abs(movement_status.distance - self.m_distance_forward_ram_dice) < 0.001 and movement_status.power == 0:
            self.is_running_dice2_forward = False

        elif self.is_running_dice2_backward and movement_status.state == 0  and abs(movement_status.distance - self.m_distance_backward_ram_dice) < 0.001 and movement_status.power == 0:
            self.is_running_dice2_backward = False

    def is_running_task(self):
        return self.is_running_task_pregate
