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


    states = ['searching','path',{'name': 'buoy', 'children' :['searchMonster','triangle','jiangshi','position','ram']},
            {'name':'garlic', 'children':['findLever','pullLever','foundTarget','findGarlic','pickUpGarlic','drop']},
            {'name': 'torpedo','children':['lookForLever','uncover','lookForOtherTarget','prevCoveredTarget','position','fire']},
            {'name' :'octagon', 'children': ['findVampire','pickUpVampire','rise']},'pingers','gate','runCompleted']
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

        transitions = [
         ['completeGateTask','gate','=','gateTaskNotCompleted'],
         ['completeGateTask','gate','path','gateTaskCompleted'],
         ['pathToNextTask','path','searching','atAssignedAngle'],
         ['pathToNextTask','path','=','notAtAssignedAngle'],
         ['identifyBuoyMonster','buoy_searchMonster','buoy_triangle','monsterFound'],
         ['identifyBuoyMonster','buoy_searchMonster','buoy_jiangshi','monsterNotFound'],
         ['findMonsterPosition','buoy_triangle','buoy_position','subInPosition'],
         ['findMonsterPosition','buoy_jiangshi','buoy_position','subInPosition'],
         ['findMonsterPosition','buoy_triangle','buoy_jiangshi','tooManyPositionAttempts'],
         ['findMonsterPosition','buoy_jiangshi','searching','tooManyPositionAttempts'],
         ['ramMonster','buoy_position','buoy_ram'],
         ['buoyTaskCompleted','buoy_ram','searching'],
         ['findLever','torpedo_lookForLever','torpedo_uncover','leverFound'],
         ['findLever','torpedo_lookForLever','torpedo_lookForOtherTarget','leverNotFound'],
         ['findLever','torpedo_lookForLever','torpedo_fire',['leverNotFound','targetNotFound']],
         ['pullLevel','torpedo_uncover','torpedo_prevCoveredTarget','leverPulled'],
         ['pullLevel','torpedo_uncover','torpedo_lookForOtherTarget','leverNotPulled'],
         ['findTargetPosition','torpedo_prevCoveredTarget','torpedo_position','foundTarget'],
         ['findTargetPosition','torpedo_lookForOtherTarget','torpedo_position','foundTarget'],
         ['findTargetPosition','torpedo_prevCoveredTarget','torpedo_fire','targetNotFound'],
         ['findTargetPosition','torpedo_lookForOtherTarget','torpedo_fire','targetNotFound'],
         ['readytoFire','torpedo_position','torpedo_fire',['subInPosition','notTooManypositioningAttempts']],
         ['readytoFire','torpedo_position','torpedo_fire','tooManypositioningAttempts'],
         ['torpedoFired','torpedo_fire','searching','torpedoWasFired'],
         ['garlicFindLever','garlic_findLever','garlic_pullLever','leverFound'],
         ['garlicFindLever','garlic_findLever','garlic_findGarlic','leverNotFound'],
         ['garlicPullLever','garlic_pullLever','garlic_findGarlic','leverPulled'],
         ['garlicPullLever','garlic_pullLever','=',['leverNotPulled','notTooManyAttempts']],
         ['garlicPullLever','garlic_pullLever','garlic_findGarlic','tooManyAttempts'],
         ['garlicFindGarlic','garlic_findGarlic','garlic_pickUpGarlic','garlicFound'],
         ['garlicFindGarlic','garlic_findGarlic','=','garlicNotFound'],
         ['garlicFindTarget','garlic_pickUpGarlic','garlic_foundTarget','targetFound'],
         ['garlicFindTarget','garlic_pickUpGarlic','=',['targetNotFound','notTooManyAttempts']],
         ['garlicPositionOverTarget','garlic_foundTarget','garlic_drop','maintainingPosition'],
         ['garlicDropped','garlic_drop','searching','garlicReleased'],
         ['octagonSearch','octagon_findVampire','octagon_pickUpVampire','vampireFound'],
         ['octagonSearch','octagon_findVampire','=','vampireNotFound'],
         ['vampirePickUp','octagon_pickUpVampire','octagon_rise','vampirePickedUp'],
         ['vampirePickUp','octagon_pickUpVampire','=','vampireNotPickedUp'],
         ['octagonToSurface','octagon_rise','runCompleted'],
         ['searchForTask',['garlic_drop','torpedo_fire','pingers','buoy_ram','buoy_jiangshi'],'searching'],
         ['searchForPath','searching','path','pathNumberUnderTwo'],
         ['searchForBuoy','searching','buoy_searchMonster','buoysTaskNotCompleted'],
         ['searchForPinger','searching','pingers','pathNumberOverTwo'],
         ['searchForTorpedo','searching','torpedo_lookForLever','torpedoTaskNotCompleted'],
         ['searchForGarlic','searching','garlic_findLever','garlicTaskNotCompleted'],
         ['searchForOctagon','searching','octagon_findVampire','octagonTaskNotCompleted']
        ]

        m = Houston()
        machine = GraphMachine(model=m, states=states,transitions=transitions,initial='gate')



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
