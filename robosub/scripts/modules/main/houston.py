import rospy
import cv2
import sys
import time
import threading
import time

import numpy as np

from threading import Thread
from collections import Counter
from itertools import combinations
from transitions import Machine

from robosub.msg import CVIn
from robosub.msg import CVOut

import modules.main.config as config

from modules.tasks.pregate import PreGate
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

from my_states import *
from device import SimpleDevice

# TODO
# add new module to houston for gui to display video
# use import frame or sample over

class Houston():
    import rospy
    import cv2
    import sys
    import time
    import threading
    import time

    import numpy as np

    from threading import Thread
    from collections import Counter
    from itertools import combinations

    from robosub.msg import CVIn
    from robosub.msg import CVOut

    import modules.main.config as config

    from modules.tasks.pregate import PreGate
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
        #states = ['Searching','pregate', 'gate', 'path', 'dice', 'path', 'slots',
        #'chip','pinger_a','roulette', 'pinger_b', 'cash_in']

        def __init__(self, navigation, task_list):
            """ To initilize Houston """
            ################ INSTANCES ################
            self.pregate = PreGate(self)
            self.gate = Gate(self)
            self.path_1 = Path(self)
            self.dice = Dice(self)
            self.path_2 = Path(self)
            self.chip = Chip(self)
            self.roulette = Roulette(self)
            self.slots = Slots(self)
            self.pinger_a = PingerA(self)
            self.pinger_b = PingerB(self)
            self.cash_in = CashIn(self)
            # self.buoy = Buoy(self)
            self.navigation = navigation
            self.cvcontroller = CVController()
            self.counts = Counter()

            self.orientation = Orientation()

            ###########################################
            ########## RUN ALL TASK QUEUE #############
            self.taskList_run_all = [
                self.pregate,
                self.gate,
                self.path_1,
                self.dice,
                self.path_2,
                self.slots
            ]

            ################ THRESHOLD VARIABLES ################
            self.task_timer = 300
            self.break_timer = 600

            ################ FLAG VARIABLES ################
            self.is_killswitch_on = False
            self.is_task_running = False

            ################ TIMER/COUNTER VARIABLES ################
            self.last_time = time.time()

            ################ TASKS LIST ################
            """
            self.tasks values listed below
            'pregate', 'gate', 'path', 'dice', 'path', 'slots', 'chip', 'pinger_a',
            'roulette', 'pinger_b', 'cash_in'
            """
            self.tasks = task_list

            ################ Transitions Library ####################
            #self.machine = Machine(model=self, states=Houston.states, initial='Searching')
            #self.machine.add_transition(trigger='pregate', source='[Searching', 'path'], dest='gate')
            #self.machine.add_transition(trigger='path', source='gate', dest='dice')
            #self.machine.add_transition(trigger='path', source='dice', dest='slots')
            #self.machine.add_transition(trigger='chip', source='slots', dest='pinger_a')
            #self.machine.add_transition(trigger='roulette', source='pinger_a', dest='pinger_b')
            #self.machine.add_transition(trigger='cash_in', source='pinger_b', dest='=')

            ################ DICTIONARIES ################
            self.taskList_num = 0
            self.taskList = [
                self.pregate,
                self.gate,
                self.path_1,
                self.dice,
                self.path_2,
                self.slots,
                self.pinger_a,
                self.chip,
                self.roulette,
                self.pinger_b,
                self.cash_in
            ]

            self.gui_states = {
                'pregate': self.pregate,
                'gate': self.gate,
                'path': self.path_1,
                'dice': self.dice
            }

            self.one_or_all_tasks = {
                'one': self.do_one_task,
                'all': self.start_all_tasks
            }

            self.gui_task_calls = {
                'pregate': 0,
                'gate': 1,
                'path_1': 2,
                'dice': 3,
                'path_2': 4,
                'slots': 5,
                'pinger_a': 6,
                'chip': 7,
                'roulette': 8,
                'pinger_b': 9,
                'cash_in': 10
            }

            ################ AUV MOBILITY VARIABLES ################
            # self.rotational_movement = {-1: }
            self.height = 1
            self.queue_direction = []
            self.rotation = 15
            self.power = 120
            self.r_power = 80

            ################ TASK THREAD ################
            self.task_thread = None
            self.all_task_loop = True

            ################ ROS VARIABLES ################
            self.r = rospy.Rate(30)  # 30hz
            self.msg = CVIn()

            ################ CURRENT TASK VARIABLE ################
            self.current_task = None
            #self.current_task = self.machine.state

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

        # start_task_from_gui ##################################################################################
        def start_task_from_gui(self, one_or_all, task_name):
            if not self.is_task_running:
                self.task_thread_start(one_or_all, self.gui_task_calls[task_name])
            else:
                print '\nTask is currently running.'
                print '\nPlease wait for task to finish or cancel'

        # start_all_tasks ##################################################################################
        def start_all_tasks(self, _):
            device = SimpleDevice() #starts machine state
            time.sleep(7)
            # self.navigation.h_nav('down', 6, 100)
            # time.sleep(5)
            # self.is_task_running = True
            self.navigation.cancel_all_nav()

            # self.all_task_loop = True
            # self.taskList_num = 0

            device.on_event('start')
            device.state #prints out what state you're currently on

            device.on_event('start_gate')
            device.state

            device.on_event('start_task')


            # while self.all_task_loop:
            #     if self.taskList_num > len(self.taskList_run_all) - 1:
            #         self.all_task_loop = False
            #         print 'no more tasks to complete'
            #
            #     # self.run_orientation()
            #
            #     # Added to show mark we are able to set orientation before hand
            #     # print 'start_orientation {}'.format(self.orientation.start_orientation)
            #     # print 'start_angle {}'.format(self.orientation.start_angle)
            #     # self.navigation.r_nav(self.orientation.start_orientation, self.orientation.start_angle, self.r_power)
            #     # self.navigation.ros_sleep(3)
            #     # self.navigation.m_nav('power', 'forward', self.power)
            #     # self.navigation.ros_sleep(3)
            #     else:
            #         self.current_taskList = self.taskList_run_all[self.taskList_num]
            #
            #         self.current_taskList.reset()
            #         print 'doing task: {}'.format(self.current_taskList.task_name)
            #         self.current_taskList.start(self.current_taskList.task_name, self.navigation, self.cvcontroller, self.power,
            #                          self.rotation)
            #
            #         if self.current_taskList.complete():
            #             self.taskList_num += 1
            #
            # self.is_task_running = False

        # run_orientation ##################################################################################
        def run_orientation(self):
            self.orientation.set_orientation(self.navigation, self.power, self.r_power)

        # do_one_task ##################################################################################
        def do_one_task(self, task_num):
            self.is_task_running = True
            self.cancel_all_nav()
            #self.navigation.cancel_h_nav()
            #self.navigation.cancel_m_nav()
            #self.navigation.cancel_r_nav()
            self.current_taskList = self.taskList[task_num]

            print '\nattempting to run task number: {}\
                   \ntask: {}'.format(task_num, self.current_taskList.task_name)

            self.current_taskList.reset()
            self.current_taskList.start(self.current_taskList.task_name, self.navigation, self.cvcontroller, self.power, self.rotation)

            self.is_task_running = False

        # stop_task ##################################################################################
        def stop_task(self):
            try:
                self.current_taskList.stop_task = True
                self.all_task_loop = False
            except:
                print 'no task currently running to stop'

            self.cancel_all_nav()
            #self.navigation.cancel_h_nav()
            #self.navigation.cancel_m_nav()
            #self.navigation.cancel_r_nav()

            # return_raw_frame ##################################################################################

        def return_raw_frame(self):
            if self.current_taskList.is_task_running:
                return self.cvcontroller.current_raw_frame()
            else:
                print 'camera is currently not running'

        # return_processed_frame ##################################################################################
        def return_processed_frame(self):
            if self.current_taskList.is_task_running:
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
            self.task_thread = Thread(target=self.one_or_all_tasks[one_or_all], args=(task_choice,))
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
            # self.get_task()
            # similar start to other classes, such as auv, and keyboard
            # self.is_killswitch_on = True
            self.navigation.start()

        # stop ##################################################################################
        def stop(self):
            # similar start to other classes, such as auv, and keyboard
            # self.is_killswitch_on = False
            self.navigation.stop()
