import rospy
import cv2
import time

from robosub.msg import CVIn
from robosub.msg import CVOut
from config.config import Config

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

# TODO houston will communicate with CV controller through ROS
# houston will interpet the coordinates and then send it to navigation
class Houston():
    # implements(Task)
    
    def __init__(self):
        """ To initilize Houston """
        self.is_killswitch_on = False
        self.config = Config()
        self.MAX_POWER = 400
        self.MID_POWER = 200
        self.MIN_POWER = 100

        self.coordinates = []
        self.task_num = 0
        self.sway_dir = 'right'
        self.sway_counter = 0
        self.task_timer = 300

        # setting class instances of the tasks to none
        self.gate = None
        self.path = None
        self.roulette = None
        self.slots = None
        self.pinger_a = None
        self.pinger_b = None
        self.cash_in = None
        self.buoy = None
        self.cap = cv2.VideoCapture(0)
        self.navigation = Navigation()

        # below is code to use rostopic cv_to_master
        self.pub = rospy.Publisher('cv_to_master', CVIn)
        #rospy.init_node('cv_talker', anonymous=True)
        self.r = rospy.Rate(30) #30hz
        self.msg = CVIn()

        # TODO must eventually move to CVController
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('video_output/gate-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (640, 480))

    def do_task(self):
        # when task_num is > 10, there will be no more tasks to complete
        if self.task_num > 10:
            print 'no more tasks to complete'

        elif self.tasks[self.task_num] == 'gate':
            if not self.gate:
                self.gate = Gate()
                    
            while not self.gate.is_gate_done:
                # TODO must eventually move to CVController
                _, frame = self.cap.read()
                self.msg.found, gate_coordinates = self.gate.detect(frame)

                if self.msg.found and gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
                    self.navigation.m_nav('power', 'forward', self.MID_POWER)

                if gate_coordinates[0] == 1 and self.msg.found:
                    self.navigation.m_nav('power', 'right', self.MID_POWER)
                elif gate_coordinates[0] == -1 and self.msg.found:
                    self.navigation.m_nav('power', 'left', self.MID_POWER)

                if gate_coordinates[1] == 1 and self.msg.found: 
                    self.navigation.h_nav('up', height, self.MID_POWER)
                elif gate_coordinates[1] == -1 and self.msg.found:
                    self.navigation.h_nav('down', height, self.MID_POWER)

                if not self.msg.found:
                    self.navigation.m_nav('power', self.sway_dir, self.MID_POWER)
                    self.sway_counter += 1

                if self.sway_counter > 60:
                    self.sway_counter = 0
                    if self.sway_dir == 'right': self.sway_dir = 'left'
                    else: self.sway_dir = 'right'

                
                if self.gate.not_found_timer > self.task_timer and not self.gate.is_gate_found:
                    self.navigation.m_nav('power', 'forward', self.MID_POWER)
                    self.gate.is_gate_done = True
                
                # TODO must eventually move to CVController
                self.out.write(frame)
                cv2.imshow('gate',frame)

                # below code is used to add data to msg to be published by ros
                self.msg.horizontal = gate_coordinates[0]
                self.msg.vertical = gate_coordinates[1]
                self.msg.distance = 1.25
                self.msg.targetType = 1.0
                #rospy.loginfo(self.msg)
                self.pub.publish(self.msg)
                self.r.sleep()

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

    def get_task(self):
        self.tasks = self.config.get_config('auv', 'tasks')
        # ['gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        # 'roulette', 'pinger_a', 'cash_in']

    def start(self):
        self.get_task()

        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = True
        self.navigation.start()
    
    def stop(self):
        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = False
        self.navigation.stop()
        