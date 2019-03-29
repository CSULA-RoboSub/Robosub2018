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




class Command:
    def execute(self): pass

class Path_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.path.task_name)
        self.path.start(self.path.task_name, self.navigation, self.cvcontroller, self.power,
                           self.rotation)

class Dice_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.dice.task_name)
        self.dice.start(self.dice.task_name, self.navigation, self.cvcontroller, self.power,
                           self.rotation)

class Slots_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.slots.task_name)
        self.slots.start(self.slots.task_name, self.navigation, self.cvcontroller, self.power,
                           self.rotation)


class Pinger_A_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.pinger_a.task_name)
        self.pinger_a.start(self.pinger_a.task_name, self.navigation, self.cvcontroller, self.power,
                           self.rotation)

class Pinger_B_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.pinger_b.task_name)
        self.pinger_b.start(self.pinger_b.task_name, self.navigation, self.cvcontroller, self.power,
                            self.rotation)

class Chip_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.chip.task_name)
        self.chip.start(self.chip.task_name, self.navigation, self.cvcontroller, self.power,
                         self.rotation)

class Roulette_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.roulette.task_name)
        self.roulette.start(self.roulette.task_name, self.navigation, self.cvcontroller, self.power,
                         self.rotation)

class Cash_In_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.cash_in.task_name)
        self.cash_in.start(self.cashi_in.task_name, self.navigation, self.cvcontroller, self.power,
                         self.rotation)

class Pregate_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.pregate.task_name)
        self.pregate.start(self.pregate.task_name, self.navigation, self.cvcontroller, self.power,
                                    self.rotation)

class Gate_Task(Command):
    def execute(self):
        print 'doing task: {}'.format(self.gate.task_name)
        self.gate.start(self.gate.task_name, self.navigation, self.cvcontroller, self.power,
                           self.rotation)

# An object that holds commands:
class Activities:
    def __init__(self):
        self.commands = []
    def add(self, command):
        self.commands.append(command)
    def run(self):
        for c in self.commands:
            c.execute()

class Gate_Act:
    def __init__(self):
        self.commands = []
    def add(self, command):
        self.commands.append(command)
    def run(self):
        for c in self.commands:
            c.execute()

#
# class CPattern(object):
#
#     act = Activities()
#     act.add(PathFinder())
#     act.add(Task1())
#     act.add(PathFinder())
#     act.add(Task2())
#     act.add(PathFinder())
#     act.add(Task3())
#     act.add(PathFinder())
#     act.add(Task4())
#     act.add(PathFinder())
#     act.run()







