import time
import rospy

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task
from robosub.msg import RControl
from robosub.msg import HControl

class PreGate(Task):
    
    def __init__(self, Houston):
        """ To initialize pregate """
        super(PreGate, self).__init__()

        ################ INSTANCES ################
        self.houston = Houston
        
        ################ THRESHOLD VARIABLES ################

        ################ FLAG VARIABLES ################
        self.stop_task = False
        self.is_complete = False

        self.is_busy = False
        self.is_running_rotation = False
        ################ TIMER VARIABLES ################

        ################ DICTIONARIES ################
        self.headings = {
            'a': 317.0,
            'b': 358.0,
            'c': 160.0,
            'd': 167.5
        }

        ################ AUV MOBILITY VARIABLES ################
        self.selected_heading = 'b'
        
        self.r_power=80
        self.h_power=100
        self.m_power=120
        self.h_depth = 5

        ################ THREAD VARIABLES ################
        self.mutex = Lock()

        ################ PREGATE VARIABLES ################

        ################ PREGATE CONSTANTS ################

        
        ################ PREGATE ROS VARIABLES ################
        rospy.Subscriber('rotation_control_status', RControl, self.r_status_callback, queue_size=100)
        rospy.Subscriber('height_control_status', HControl, self.h_status_callback, queue_size=100)
    # reset ##################################################################################
    def reset(self): 
        self.stop_task = False
        self.is_complete = False

        self.is_busy = False
        self.is_running_rotation = False
    
    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        self.local_cvcontroller = cvcontroller
        # cvcontroller.camera_direction = 'forward'
        # cvcontroller.start(task_name)
        # count = 0
        self.mutex.acquire()

        self.is_busy = False
        self.is_running_rotation = False

        navigation.h_nav('down', 3.5, self.h_power)
        
        while not self.stop_task and not self.complete():
            if not self.is_busy and not self.is_running_rotation:
                # print('in rotation')
                self.is_busy = True
                self.is_running_rotation = True
                #run rotation
                print(self.headings['a'])
                direction, degree = navigation.waypoint.get_directions_with_heading(self.headings['a'])
                # print('direction: {} degree: {}'.format(str(direction), str(degree)))
                #set is running rotation
                navigation.r_nav(direction, degree, self.r_power)

            elif self.is_busy and not self.is_running_rotation:
                # print('in move')
                #run forward
                navigation.m_nav('power', 'forward', self.m_power)
                #set complete
                self.is_complete = True
            pass
        # cvcontroller.stop()
        print 'PreGate Finish'
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
        if self.is_busy and self.is_running_rotation:
            if rotation_status.state == 1:
                #set is running rotation false
                self.is_running_rotation = False

    def h_status_callback(self, height_status):
        if height_status.state == 1:
            self.is_complete = True
