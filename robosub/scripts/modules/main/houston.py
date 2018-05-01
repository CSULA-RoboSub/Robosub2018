import rospy
import math
import sys

from robosub.msg import CVIn
from robosub.msg import CVOut
from config.config import Config

'''
sys.path.append('../controller/')
sys.path.append('../control/')
sys.path.append('../tasks/')
'''

from modules.tasks.gate import Gate
from modules.tasks.path import Path
from modules.tasks.dice import Dice
from modules.tasks.roulette import Roulette
from modules.tasks.slots import Slots
from modules.tasks.pinger_a import PingerA
from modules.tasks.pinger_b import PingerB
from modules.tasks.cash_in import CashIn
from modules.tasks.buoy import Buoy
from modules.tasks.task import Task

from modules.control.navigation import Navigation

class Houston():
    # implements(Task)
    
    def __init__(self):
        """ To initilize Houston """
        self.config = Config()
        self.coordinates = []
        self.task_num = 0

        self.gate = None
        self.path = None
        self.roulette = None
        self.slots = None
        self.pinger_a = None
        self.pinger_b = None
        self.cash_in = None
        self.buoy = None

        self.navigation = Navigation()

        print 'houston init method successful'

    def do_task(self):
        """ the follow ifs' are for testing methods, will modify soon """
        '''id = argh
        constructor = globals()[id]
        instance = constructor()
        if not self.instance:
            self.instance = instance
        '''
        if self.task_num > 10:
            print 'no more tasks to complete'

        elif self.tasks[self.task_num] == 'gate':
            if not self.gate:
                    self.gate = Gate()
                    
            while not self.gate.is_gate_done:

                if not self.gate.is_gate_found or not self.gate.not_found_timer > 240:
                    found, gate_coordinates = self.detect_task(self.gate)
                
                if self.gate.is_gate_found:
                    found, gate_coordinates = self.gate.navigate()
                
                if self.not_found_timer > 240:
                    self.task_num += 1

                navigation.m_nav(gate_coordinates)

                #return found, gate_coordinates
            # TODO use while statement until task is completed or done

            print 'do_task gate'
            self.task_num += 1

        elif self.tasks[self.task_num] == 'path':
            '''if not self.path:
                self.path = Path()
            else:
                print self.path.detect()
            '''
            print 'do_task path'
            self.task_num += 1

        elif self.tasks[self.task_num] == 'dice':
            print 'do_task dice'
            self.task_num += 1

        elif self.tasks[self.task_num] == 'chip':
            print 'do_task chip'
            self.task_num += 1

        elif self.tasks[self.task_num] == 'slots':
            print 'do_task slots'
            self.task_num += 1

        elif self.tasks[self.task_num] == 'pinger_a':
            print 'do_tasks pinger_a'
            self.task_num += 1

        elif self.tasks[self.task_num] == 'pinger_b':
            print 'do_task pinger_b'
            self.task_num += 1

        elif self.tasks[self.task_num] == 'roulette':
            print 'do_task roulette'
            self.task_num += 1

        elif self.tasks[self.task_num] == 'cash_in':
            print 'do_task cash_in'
            self.task_num += 1

        '''setattr(self, argh, self.get_thing)'''

        
    def detect_task(self, task):
        print 'testing poly'
        task.detect()

    def gate(self):
        pass

    def start(self):
        #self.navigation.start()
        self.get_task()
    
    def stop(self):
        #self.navigation.stop()
        print 'stopping houston'

    def get_task(self):
        self.tasks = self.config.get_config('auv', 'tasks')
        # ['gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        # 'roulette', 'pinger_a', 'cash_in']
        print 'test printing tasks'
        print self.tasks


