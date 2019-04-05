import rospy
import cv2
import sys
import time
import threading
import time
import random

import numpy as np

from threading import Thread
from collections import Counter
from itertools import combinations
from transitions.extensions import MachineFactory
from IPython.display import Image, display, display_png
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

class Houston():

    states = ['searching','path','buoys','garlic',{'name': 'torpedo','children':['uncover','prime','fired']},'octagon','pingers','gate','finish']
    def __init__(self, navigation):
        self.pregate = PreGate(self)
        self.gate = Gate(self)
        self.path = Path(self)
        self.navigation = navigation
        self.cvcontroller = CVController()
        self.counts = Counter()
        self.task_timer = 300
        self.break_timer = 600    

        ################ AUV MOBILITY VARIABLES ################
        # self.rotational_movement = {-1: }
        self.height = 1
        self.queue_direction = []
        self.rotation = 15
        self.power = 120
        self.r_power = 80

        self.is_killswitch_on = False

        self.last_time = time.time()
        m = Houston()
        machine = Machine(model=m, states=states, initial='searching')


        transitions = [
        ['goToGateTask','searching','gate'],
        ['completeGateTask','gate','=','gateTaskNotCompleted'],
        ['completeGateTask','gate','path','gateTaskCompleted'],
        ['pathToBuoyTask','path','buoys','atAssignedAngle'],
        ['pathToBuoyTask','path','=','notAtAssignedAngle'],
        ['completeBuoyTask','buoys','=','buoysTaskNotCompleted'],
        ['completeBuoyTask','buoys','path','buoysTaskCompleted'],
        ['pathToTorpedoTask','path','torpedo_uncover','atAssignedAngle'],
        ['uncoverBox','torpedo_uncover','torpedo_prime', 'targetUncovered'],
        ['uncoverBox','torpedo_uncover','=','targetCovered'],
        ['primeTorpedo','torpedo_prime','=','torpedoNotPrimedCorrectly'],
        ['fireTorpedo','torpedo_prime','torpedo_fired','torpedoPrimedCorrectly'],
        ['completeGarlicTask','garlic','=','garlicTaskNotCompleted'],
        ['pingerToOctagon','pingers','octagon'],
        ['pingerToGarlic','pingers','garlic'],
        ['usePingersFromTorpedo','torpedo_fired','pingers','torpedoFired'],
        ['usePingersFromGarlic','garlic','pingers','garlicTaskCompleted'],
        ['octagontoSurface','octagon','finish','octagonTaskCompleted'],
        ['completeOctagonTask','octagon','=']
        ]

    def gateTaskNotCompleted(self):
        return True

    def buoysTaskNotCompleted(self):
        return True

    def gateTaskCompleted(self):
        return True

    def buoysTaskCompleted(self):
        return True

    def atAssignedAngle(self):
        return True

    def notAtAssignedAngle(self):
        return True

    def torpedoPrimedCorrectly(self):
        return True

    def torpedoNotPrimedCorrectly(self):
        return True

    def torpedoFired(self):
        return True

    def garlicTaskNotCompleted(self):
        return True

    def garlicTaskCompleted(self):
        return True

    def octagonTaskNotCompleted(self):
        return True

    def octagonTaskCompleted(self):
        return True

    def targetCovered(self):
        return True

    def targetUncovered(self):
        return True
