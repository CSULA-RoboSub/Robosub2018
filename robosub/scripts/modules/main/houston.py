import rospy
import math
import sys

from robosub.msg import CVIn
from robosub.msg import CVOut
#from robosub.scripts.config.config import Config

sys.path.append('../controller/')
sys.path.append('../control/')
sys.path.append('../tasks/')

from gate import Gate
from path import Path
from roulette import Roulette
from slots import Slots
from pinger_a import PingerA
from pinger_b import PingerB
from cash_in import CashIn
from buoy import Buoy
from task import Task

from navigation import Navigation

class Houston():
    #implements(Task)
    
    """self.gate = None
    self.path = None
    self.roulette = None
    self.slots = None
    self.pinger_a = None
    self.pinger_b = None
    self.cash_in = None
    self.buoy = None
    """

    def __init__(self):
        """ To initilize Houston """

        self.tasks = []
        self.coordinates = []

        #the follow variables are just for testing purpose
        self.temp_task_test = ['gate', 'buoy', 'dice']
        self.task_num = 0

        self.gate = None
        self.path = None
        self.roulette = None
        self.slots = None
        self.pinger_a = None
        self.pinger_b = None
        self.cash_in = None
        self.buoy = None

        self.cv_controller = CVController()
        self.navigation = Navigation()

        print('houston init method successful')

    def do_task(self, argh):
        """ the follow ifs' are for testing methods, will modify soon """
        if argh is 'gate':
            if not self.gate:
                self.gate = Gate()
            else:
                print(self.gate.detect())

        if argh is 'path':
            print('do_task path')
        if argh is 'dice':
            print('do_task dice')
        if argh is 'chip':
            print('do_task chip')
        if argh is 'slots':
            print('do_task slots')
        if argh is 'pinger_a':
            print('do_tasks pinger_a')
        if argh is 'pinger_b':
            print('do_task pinger_b')
        if argh is 'roulette':
            print('do_task roulette')
        if argh is 'cash_in':
            print('do_task cash_in')
        
    def detect(self):
        pass

    def gate(self):
        pass

    def start(self):
        self.cv_controller.start()
        self.navigation.start()
    
    def stop(self):
        self.cv_controler.stop()
        self.navigation.stop()
