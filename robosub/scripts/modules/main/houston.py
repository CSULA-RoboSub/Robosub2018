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
        self.coordinates = []

        # will eventually move variables to task modules
        self.task_timer = 300
        self.last_time = time.time()

        self.multiplier = 10

        self.rotation = int(3) * self.multiplier
        self.power = int(20) * self.multiplier

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

        #self.rotational_movement = {-1: }
        self.height = 5

        # TODO move to CVcontroller
        self.cap = cv2.VideoCapture(0)

        # init_node only needs to be ran once, which is already called in auv.py
        #rospy.init_node('cv_talker', anonymous=True)
        self.r = rospy.Rate(30) #30hz
        self.msg = CVIn()

    def do_task(self):
        # when state_num is > 10, there will be no more tasks to complete
        if self.state_num > 10:
            print 'no more tasks to complete'
            
        break_loop = 0
        self.state = self.states[self.state_num]

        # TODO must eventually move to CVController
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('video_output/' + self.tasks[self.state_num] + '-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (640, 480))

        while not self.state.is_detect_done:
            _, frame = self.cap.read()
            self.out.write(frame)
            self.msg.found, coordinates = self.state.detect(frame)

            self.last_reading = coordinates

            # TODO must eventually move to CVController
            cv2.imshow('gate',frame)
            key = cv2.waitKey(1) & 0xFF

            # if the `q` key is pressed, break from the loop
            if key == ord("q"):
                self.navigation.cancel_h_nav()
                self.navigation.cancel_r_nav()
                self.navigation.cancel_m_nav()
                break

            if (time.time()-self.last_time > 1):
                self.last_time = time.time()

                self.state.navigate(self.navigation, self.msg.found, self.last_reading, self.power, self.rotation)
                
                """break_loop used for temp breaking of loop"""
                print 'press q to quit task or wait 30 secs'
                break_loop += 1
                if break_loop >= 30:
                    break

        if self.state.is_detect_done:
            self.state_num += 1

        self.navigation.cancel_h_nav()
        self.navigation.cancel_r_nav()
        self.navigation.cancel_m_nav()
        
        #TODO will be used to release the cap(videocapture) if needed
        # must initialize cap again if we plan to use this
        #cap.release()

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