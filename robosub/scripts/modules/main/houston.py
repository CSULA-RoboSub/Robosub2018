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
from cv_controller import CVController

class Houston():
    
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

        self.detectgate = None
        self.detectpath = None
        self.detectdice = None
        self.detectbuoy = None
        self.detectroulette = None
        self.detectdice = None

        self.is_complete_first_die = False
        self.is_complete_second_die = False

        self.is_buoy_found = False
        self.is_gate_found = False
        self.is_path_found = False
        self.is_dice_found = False
        self.is_chip_found = False
        self.is_roulette_found = False
        self.is_slots_found = False
        self.is_pinger_a_found = False
        self.is_pinger_b_found = False
        self.is_cash_in_found = False

        self.is_gate_done = False
        self.is_dice_done = False
        self.is_roulette_done = False
        self.is_cash_in_done = False

        self.found_timer = 0
        self.gate_circle_loc = 0

        self.cv_controller = CVController()
        self.navigation = Navigation()

        print('houston init method successful')

    def do_task(self, argh):
        """ the follow ifs' are for testing methods, will modify soon """
        if argh is 'gate':
            pass
        if argh is 'path':
            pass
        if argh is 'dice':
            pass
        if argh is 'chip':
            pass
        if argh is 'slots':
            pass
        if argh is 'pinger_a':
            pass
        if argh is 'pinger_b':
            pass
        if argh is 'roulette':
            pass
        if argh is 'cash_in':
            pass
        

    def gate(self):
        pass

    def start(self):
        self.cv_controller.start()
        self.navigation.start()
    
    def stop(self):
        self.cv_controler.stop()
        self.navigation.stop()
