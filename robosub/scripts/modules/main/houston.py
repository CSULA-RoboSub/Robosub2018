import rospy
import cv2
import time

from robosub.msg import CVIn
from robosub.msg import CVOut
from config.config import Config
# TODO create new msg for Task and CVData
#from robosub.msg import Task
#from robosub.msg import CVData

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

from modules.control.navigation import Navigation

#will need to move CVControll
from modules.controller.cv_controller import CVController

# TODO houston will communicate with CV controller through ROS
# houston will interpet the coordinates and then send it to navigation
class Houston():
    # implements(Task)
    
    def __init__(self):
        """ To initilize Houston """
        self.is_killswitch_on = False
        self.navigation = Navigation()
        self.config = Config()
        self.MAX_POWER = 400
        self.MID_POWER = 200
        self.MIN_POWER = 100
        self.depth = -1
        self.rotation_direction = 'right'
        self.multiplier = 40
        self.r_multiplier = 18
        self.rotation = int(10) * self.r_multiplier

        self.coordinates = []

        # will eventually move variables to task modules
        self.sway = {0: 'right', 1: 'left'}
        self.task_timer = 300
        self.last_time = time.time()

        # setting class instances of the tasks to none
        # to be used to prevent mutiple instances of same class
        self.gate = Gate(Houston)
        self.path_1 = Path(Houston)
        self.dice = Dice(Houston)
        self.path_2 = Path(Houston)
        self.chip_1 = Chip(Houston)        
        self.chip_2 = Chip(Houston)
        self.roulette = Roulette(Houston)
        self.slots = Slots(Houston)
        self.pinger_a = PingerA(Houston)
        self.pinger_b = PingerB(Houston)
        self.cash_in = CashIn(Houston)
        #self.buoy = Buoy(Houston)

        """
        self.tasks values listed below
        'gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        'roulette', 'pinger_a', 'cash_in'
        """
        self.state_num = 0
        self.states = [self.gate, self.path_1, self.dice, self.chip_1, self.path_2, 
                        self.slots, self.chip_2, self.pinger_a, self.roulette,
                        self.pinger_b, self.cash_in]
    
        self.last_reading = []
        self.mState = {'off': 0, 'power': 1, 'distance': 2, 'front_cam_center': 3, 'bot_cam_center': 4, 'motor_time': 5}
        self.horizontal_move = {0: 'none', -1: 'left', 1: 'right'}
        self.vertical_movement = {-1: 'down', 0: 'staying', 1: 'up'}

        self.power = 'power'
        self.move_forward = 'forward'
        self.depth_change = 50
        #self.rotational_movement = {-1: }
        self.height = 5

        # TODO move to CVcontroller
        self.cap = cv2.VideoCapture(0)

        # init_node only needs to be ran once, which is already called in auv.py
        #rospy.init_node('cv_talker', anonymous=True)
        self.r = rospy.Rate(30) #30hz
        self.msg = CVIn()

        # TODO must eventually move to CVController
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('video_output/gate-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (640, 480))

    def do_task(self):
        #added just to make sure all the motors are off before starting a new task
        self.navigation.m_nav('off', 'none', 0)
        self.navigation.r_nav('staying', 0, 0)
        self.navigation.h_nav('staying', 0, 0)

        # when state_num is > 10, there will be no more tasks to complete
        if self.state_num > 10:
            print 'no more tasks to complete'
            
        break_loop = 0
        self.state = self.states[self.state_num]

        while not self.state.is_detect_done:
            _, frame = self.cap.read()
            self.msg.found, coordinates = self.state.detect(frame)

            self.last_reading = coordinates
            if (time.time()-self.last_time > 1):
                print 'inside 1 second loop'
                self.last_time = time.time()

                if self.msg.found:
                    if self.last_reading == [0,0]:
                        self.navigation.m_nav(self.power, self.move_forward, self.MAX_POWER)
                    else:
                        self.navigation.m_nav(self.power, self.horizontal_move[self.last_reading[0]], self.MAX_POWER)
                        self.navigation.h_nav(self.vertical_movement[self.last_reading[1]], self.depth_change, self.MAX_POWER)
                else:
                        self.navigation.r_nav(self.rotation_direction, self.rotation, self.MAX_POWER)

                self.navigation.ros_sleep(1)
                
                """break_loop used for temp breaking of loop"""
                break_loop += 1
                if break_loop % 5 == 0:
                    self.sway_counter += 1
                if break_loop >= 10:
                    break

        if self.state.is_detect_done:
            pass

        #self.state.navigate()
        '''
        self.state_num will be changed to change the state/task
        keeping commented for now until gate can complete successfully
        '''
        #self.state_num += 1
                    
        '''print self.state.detect()
        self.state.bail_task()
        self.state.restart_task()
        self.state_num += 1
        self.state = self.states[self.state_num]
        self.state.detect()
            
        # TODO must eventually move to CVController
        self.out.write(frame)
        cv2.imshow('gate',frame)'''


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
        