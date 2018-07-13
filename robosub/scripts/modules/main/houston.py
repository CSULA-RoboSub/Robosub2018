import rospy
import cv2
import sys
import time
import gi
import threading

import numpy as np

from threading import Thread
from collections import Counter
from itertools import combinations

from robosub.msg import CVIn
from robosub.msg import CVOut

from config.config import Config

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

from modules.controller.cv_controller import CVController

# TODO
# add new module to houston for gui to display video
# use import frame or sample over

class Houston():
    # implements(Task)
    
    def __init__(self, navigation):
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
        self.config = Config()
        self.counts = Counter()

        ################ THRESHOLD VARIABLES ################
        self.task_timer = 300
        self.break_timer = 600

        ################ FLAG VARIABLES ################
        self.is_killswitch_on = False

        ################ TIMER/COUNTER VARIABLES ################
        self.last_time = time.time()

        ################ DICTIONARIES ################
        """
        self.tasks values listed below
        'gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        'roulette', 'pinger_a', 'cash_in'
        """
        self.state_num = 0
        self.states = [self.gate, 
                        self.path_1, 
                        self.dice, 
                        self.chip_1, 
                        self.path_2,
                        self.slots, 
                        self.chip_2, 
                        self.pinger_a, 
                        self.roulette, 
                        self.pinger_b, 
                        self.cash_in]

        ################ AUV MOBILITY VARIABLES ################
        #self.rotational_movement = {-1: }
        self.height = 1
        self.queue_direction = []
        self.rotation = 15
        self.power = 120

        ################ TASK THREAD ################
        self.task_thread = None

        ################ ROS VARIABLES ################
        self.r = rospy.Rate(30) #30hz
        self.msg = CVIn()

        ################ CURRENT TASK VARIABLE ################
        self.current_task = None

    # do_task ##################################################################################
    def start_all_tasks(self):
        if self.state_num > 10:
            print 'no more tasks to complete'
        
        self.state = self.states[self.state_num]
        if not self.state.is_task_running:
            self.state.reset()
            print 'doing task: {}'.format(self.tasks[self.state_num])
            self.task_thread_start(self.state, self.tasks[self.state_num], self.navigation, self.cvcontroller, self.power, self.rotation)
            self.navigation.cancel_h_nav()
            self.navigation.cancel_m_nav()
            self.navigation.cancel_r_nav()
        else:
            print '\nTask is currently running.'
            print '\nPlease wait for task to finish or cancel'

    # do_one_task ##################################################################################
    def do_one_task(self, task_num):
        print 'currently running task {}'.format(task_num)
        self.state = self.states[task_num]
        if not self.state.is_task_running:
            self.state.reset()
            print 'doing task: {}'.format(self.tasks[task_num])
            self.task_thread_start(self.state, self.tasks[task_num], self.navigation, self.cvcontroller, self.power, self.rotation)
            self.navigation.cancel_h_nav()
            self.navigation.cancel_m_nav()
            self.navigation.cancel_r_nav()
        else:
            print '\nTask is currently running.'
            print '\nPlease wait for task to finish or cancel'

    # stop_task ##################################################################################
    def stop_task(self):
        # self.state = self.states[self.state_num]
        self.state.stop_task = True
        self.navigation.cancel_h_nav()
        self.navigation.cancel_m_nav()
        self.navigation.cancel_r_nav()

    # stop_one_task ##################################################################################
    def stop_one_task(self, task_num):
        # self.state = self.states[task_num]
        self.state.stop_task = True
        self.navigation.cancel_h_nav()
        self.navigation.cancel_m_nav()
        self.navigation.cancel_r_nav()

    # task_thread_start ##################################################################################
    def task_thread_start(self, task_call, task_name, navigation, cvcontroller, power, rotation):
        self.reset_thread()
        self.task_thread = Thread(target = task_call.start, args = (task_name, navigation, cvcontroller, power, rotation))
        self.task_thread.start()

    # reset_thread ##################################################################################
    def reset_thread(self):
        if self.task_thread:
            self.task_thread = None

    # get_task ##################################################################################
    def get_task(self):
        self.tasks = self.config.get_config('auv', 'tasks')
        # ['gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        # 'roulette', 'pinger_a', 'cash_in']

    # start ##################################################################################
    def start(self):
        self.get_task()
        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = True
        self.navigation.start()
    
    # stop ##################################################################################
    def stop(self):
        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = False
        self.navigation.stop()