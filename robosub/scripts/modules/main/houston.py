import rospy
import cv2
import sys
import time
# import gi
import threading

import numpy as np

from threading import Thread
from collections import Counter
from itertools import combinations

from robosub.msg import CVIn
from robosub.msg import CVOut

import modules.main.config as config

from modules.tasks.gate import Gate
from modules.tasks.path import Path
from modules.tasks.dice import Dice
from modules.tasks.chip import Chip
from modules.tasks.roulette import Roulette
from modules.tasks.slots import Slots
from modules.tasks.pinger_a import PingerA
from modules.tasks.pinger_b import PingerB
from modules.tasks.cash_in import CashIn
from modules.tasks.buoy import Buoy
from modules.tasks.task import Task

from modules.tasks.orientation import Orientation

from modules.controller.cv_controller import CVController

# TODO
# add new module to houston for gui to display video
# use import frame or sample over

class Houston():
    # implements(Task)
    
    def __init__(self, navigation, task_list):
        """ To initilize Houston """
        ################ INSTANCES ################
        self.gate = Gate(self)
        self.path_1 = Path(self)
        self.dice = Dice(self)
        self.path_2 = Path(self)
        self.chip_1 = Chip(self)        
        self.chip_2 = Chip(self)
        self.roulette = Roulette(self)
        self.slots = Slots(self)
        self.pinger_a = PingerA(self)
        self.pinger_b = PingerB(self)
        self.cash_in = CashIn(self)
        #self.buoy = Buoy(self)
        self.navigation = navigation
        self.cvcontroller = CVController()
        self.counts = Counter()
        
        self.orientation = Orientation()

        ################ THRESHOLD VARIABLES ################
        self.task_timer = 300
        self.break_timer = 600

        ################ FLAG VARIABLES ################
        self.is_killswitch_on = False
        self.is_task_running = False

        ################ TIMER/COUNTER VARIABLES ################
        self.last_time = time.time()

        ################ TASKS LIST ################
        self.tasks = task_list

        ################ DICTIONARIES ################
        """
        self.tasks values listed below
        'gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        'roulette', 'pinger_a', 'cash_in'
        """
        self.state_num = 0
        # self.states = [
        #     self.gate, 
        #     self.path_1, 
        #     self.dice, 
        #     self.chip_1, 
        #     self.path_2,
        #     self.slots, 
        #     self.chip_2, 
        #     self.pinger_a, 
        #     self.roulette, 
        #     self.pinger_b, 
        #     self.cash_in
        # ]

        self.states = [
            self.gate, 
            self.path_1, 
            self.dice
        ]

        self.one_or_all_tasks = {
            'one': self.do_one_task,
            'all': self.start_all_tasks
        }

        ################ AUV MOBILITY VARIABLES ################
        #self.rotational_movement = {-1: }
        self.height = 1
        self.queue_direction = []
        self.rotation = 15
        self.power = 120
        self.r_power = 80

        ################ TASK THREAD ################
        self.task_thread = None
        self.all_task_loop = True

        ################ ROS VARIABLES ################
        self.r = rospy.Rate(30) #30hz
        self.msg = CVIn()

        ################ CURRENT TASK VARIABLE ################
        self.current_task = None

    # reset ##################################################################################
    def reset(self):
        self.is_task_running = False
        self.all_task_loop = True
        self.orientation.reset()

    # print_task ##################################################################################
    def print_tasks(self):
        counter = 0
        for i in self.tasks:
            print '{}: {}'.format(counter, i)
            counter += 1

    # start_task ##################################################################################
    def start_task(self, one_or_all, task_choice):
        if not self.is_task_running:
            self.task_thread_start(one_or_all, task_choice)
        else:
            print '\nTask is currently running.'
            print '\nPlease wait for task to finish or cancel'

    # start_all_tasks ##################################################################################
    def start_all_tasks(self, _):
        self.is_task_running = True
        # self.navigation.cancel_h_nav()
        # self.navigation.cancel_m_nav()
        # self.navigation.cancel_r_nav()
        self.all_task_loop = True
        self.state_num = 0
        while self.all_task_loop:
            if self.state_num > len(self.states)-1:
                self.all_task_loop = False
                print 'no more tasks to complete'
            
            # self.run_orientation()

            # Added to show mark we are able to set orientation before hand
            # print 'start_orientation {}'.format(self.orientation.start_orientation)
            # print 'start_angle {}'.format(self.orientation.start_angle)
            # self.navigation.r_nav(self.orientation.start_orientation, self.orientation.start_angle, self.r_power)
            # self.navigation.ros_sleep(3)
            # self.navigation.m_nav('power', 'forward', self.power)
            # self.navigation.ros_sleep(3)
            else:
                self.state = self.states[self.state_num]

                self.state.reset()
                print 'doing task: {}'.format(self.tasks[self.state_num])
                self.state.start(self.tasks[self.state_num], self.navigation, self.cvcontroller, self.power, self.rotation)

                if self.state.complete():
                    self.state_num += 1
                
        self.is_task_running = False

    # run_orientation ##################################################################################
    def run_orientation(self):
        self.orientation.set_orientation(self.navigation, self.power, self.r_power)


    # do_one_task ##################################################################################
    def do_one_task(self, task_num):
        self.is_task_running = True
        self.navigation.cancel_h_nav()
        self.navigation.cancel_m_nav()
        self.navigation.cancel_r_nav()
        print '\nattempting to run task number: {}\
               \ntask: {}'.format(task_num, self.tasks[task_num])

        self.state = self.states[task_num]
        self.state.reset()
        self.state.start(self.tasks[task_num], self.navigation, self.cvcontroller, self.power, self.rotation)
        
        self.is_task_running = False

    # stop_task ##################################################################################
    def stop_task(self):
        try:
            self.state.stop_task = True
            self.all_task_loop = False
        except:
            print 'no task currently running to stop'

        self.navigation.cancel_h_nav()
        self.navigation.cancel_m_nav()
        self.navigation.cancel_r_nav()    

    # return_raw_frame ##################################################################################
    def return_raw_frame(self):
        if self.state.is_task_running:
            return self.cvcontroller.current_raw_frame()
        else:
            print 'camera is currently not running'

    # return_processed_frame ##################################################################################
    def return_processed_frame(self):
        if self.state.is_task_running:
            return self.cvcontroller.current_processed_frame()
        else:
            print 'camera is currently not running'

    # task_thread_start ##################################################################################
    # def task_thread_start(self, task_call, task_name, navigation, cvcontroller, power, rotation):
    #     self.reset_thread()
    #     self.task_thread = Thread(target = task_call.start, args = (task_name, navigation, cvcontroller, power, rotation))
    #     # self.task_thread = Thread(target = self.one_or_all_tasks.start, args = ())
    #     self.task_thread.start()
    def task_thread_start(self, one_or_all, task_choice):
        self.reset_thread()
        self.task_thread = Thread(target = self.one_or_all_tasks[one_or_all], args = (task_choice,))
        self.task_thread.start()

    # reset_thread ##################################################################################
    def reset_thread(self):
        if self.task_thread:
            self.task_thread = None

    # TODO currently unused. will remove eventually
    # get_task ##################################################################################
    def get_task(self):
        self.tasks = config.get_config('auv', 'tasks')
        # ['gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        # 'roulette', 'pinger_a', 'cash_in']

    # start ##################################################################################
    def start(self):
        #self.get_task()
        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = True
        self.navigation.start()
    
    # stop ##################################################################################
    def stop(self):
        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = False
        self.navigation.stop()