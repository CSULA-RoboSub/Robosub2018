import rospy
import cv2
import sys
import time
import gi
import threading

from threading import Thread
import numpy as np

# gi.require_version("Tcam", "0.1")
# gi.require_version("Gst", "1.0")

# from gi.repository import Tcam, Gst, GLib

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

#will need to move CVControll
from modules.controller.cv_controller import CVController

from collections import Counter
from itertools import combinations

class Houston():
    # implements(Task)
    
    def __init__(self, navigation):
        """ To initilize Houston """
        ################ INSTANCES ################
        self.gate = Gate(self)
        self.path_1 = Path(self)
        self.dice = Dice(self)
        self.path_2 = Path(self)
        self.chip_1 = Chip(self)        
        self.chip_2 = Chip(self)
        self.roulette = Roulette(self)
        self.slots = Slots(self)
        self.pinger_a = PingerA(self)
        self.pinger_b = PingerB(self)
        self.cash_in = CashIn(self)
        #self.buoy = Buoy(self)
        self.navigation = navigation
        self.cvcontroller = CVController()
        self.config = Config()
        self.counts = Counter()

        ################ THRESHOLD VARIABLES ################
        self.task_timer = 300
        self.break_timer = 600

        ################ FLAG VARIABLES ################
        self.is_killswitch_on = False

        ################ TIMER/COUNTER VARIABLES ################
        self.last_time = time.time()

        ################ DICTIONARIES ################
        """
        self.tasks values listed below
        'gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        'roulette', 'pinger_a', 'cash_in'
        """
        self.state_num = 0
        self.states = [self.gate, 
                        self.path_1, 
                        self.dice, 
                        self.chip_1, 
                        self.path_2,
                        self.slots, 
                        self.chip_2, 
                        self.pinger_a, 
                        self.roulette, 
                        self.pinger_b, 
                        self.cash_in]

        ################ AUV MOBILITY VARIABLES ################
        #self.rotational_movement = {-1: }
        self.height = 1
        self.queue_direction = []
        self.rotation = 15
        self.power = 120

        # TODO move to CVcontroller
        # self.cap = cv2.VideoCapture(0)

        # init_node only needs to be ran once, which is already called in auv.py
        #rospy.init_node('cv_talker', anonymous=True)
        self.r = rospy.Rate(30) #30hz
        self.msg = CVIn()
        self.sample = None
        # self.pipeline = None
        # self.loop = GLib.MainLoop()
        # self.thread = None
        self.task_thread = None

    # do_task ##################################################################################
    def do_task(self):
        
        # self.thread=Thread(target=self.do_gate)
        # self.thread.start()
        # try:
        #     self.do_gate()
        # except KeyboardInterrupt:
        #     print('keyboard interrupt on cv')
        #     self.state.is_detect_done = True
        
        # self.state.reset()
        # self.start_loop()
        if self.state_num > 10:
            print 'no more tasks to complete'
        
        self.state = self.states[self.state_num]
        if not self.state.is_task_running:
            self.state.reset()
            print 'doing task: {}'.format(self.tasks[self.state_num])
            self.task_thread_start(self.state, self.tasks[self.state_num], self.navigation, self.cvcontroller, self.power, self.rotation)
            self.navigation.cancel_h_nav()
            self.navigation.cancel_m_nav()
            self.navigation.cancel_r_nav()
        else:
            print 'Task is currently running.'
            print '\nPlease wait for task to finish or cancel'

    # stop_task ##################################################################################
    def stop_task(self):
        self.state = self.states[self.state_num]
        self.state.stop_task = True
        self.navigation.cancel_h_nav()
        self.navigation.cancel_m_nav()
        self.navigation.cancel_r_nav()

    # task_thread_start ##################################################################################
    def task_thread_start(self, task_call, task_name, navigation, cvcontroller, power, rotation):
        self.reset_thread()
        self.task_thread = Thread(target = task_call.start, args = (task_name, navigation, cvcontroller, power, rotation))
        self.task_thread.start()

    # reset_thread ##################################################################################
    def reset_thread(self):
        if self.task_thread:
            self.task_thread = None

    # TODO follow method not used anymore, will be removed soon after testing
    def do_gate(self):
        # when state_num is > 10, there will be no more tasks to complete
        if self.state_num > 10:
            print 'no more tasks to complete'
            
        break_loop = 0
        self.state = self.states[self.state_num]
        print("setup pipeline")
        self.setupPipline()
        self.thread=Thread(target=self.start_loop)
        self.thread.start()
        # TODO must eventually move to CVController
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.outraw = cv2.VideoWriter('video_output/raw' + self.tasks[self.state_num] + '-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (744, 480))
        self.outprocessed = cv2.VideoWriter('video_output/processed' + self.tasks[self.state_num] + '-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (744, 480))
        self.state.gate_maneuver.sweep_forward=0
        while not self.state.is_detect_done and not break_loop > self.break_timer:

            # _, frame = self.cap.read()
            #if a sample exists run cv
            if self.sample:
                # print("have sample")
                buf = self.sample.get_buffer()

                caps = self.sample.get_caps()
                width = caps[0].get_value("width")
                height = caps[0].get_value("height")
                try:
                    res, mapinfo = buf.map(Gst.MapFlags.READ)
                    # actual image buffer and size
                    # data = mapinfo.data
                    # size = mapinfo.size

                    # Create a numpy array from the data
                    img_array = np.asarray(bytearray(mapinfo.data), dtype=np.uint8)

                    # Give the array the correct dimensions of the video image
                    frame = img_array.reshape((height, width, 3))
                    # print(type(frame))

                    # self.outraw.write(frame)
                    # self.msg.found, coordinates = self.state.detect(frame)
                    # self.outprocessed.write(frame)

                    # self.last_reading.append(coordinates)
                    self.outraw.write(frame)
                    self.msg.found, coordinates, gate_shape, width_height = self.state.detect(frame)
                    self.outprocessed.write(frame)

                    self.show_img(frame)

                except KeyboardInterrupt:
                    self.state.is_detect_done = True
                    # raise
                finally:
                    buf.unmap(mapinfo)
                    
                # if self.msg.found:
                self.queue_direction.append(coordinates)

                # TODO must eventually move to CVController
                # try:
                #     cv2.imshow(self.tasks[self.state_num],frame)
                # except Exception as e:
                #     print(e)
                # cv2.imshow('kill window', self.img)
                # key = cv2.waitKey(1) & 0xFF

                # if the `q` key is pressed, break from the loop
                # if key == ord("q"):
                #     self.navigation.cancel_h_nav()
                #     self.navigation.cancel_r_nav()
                #     self.navigation.cancel_m_nav()
                #     break

                # will run through whenever at least 1 second has passed
                if (time.time()-self.last_time > 0.05):# and not self.msg.found):
                    # most_occur_coords = self.get_most_occur_coordinates(self.queue_direction, self.counts)
                    try:
                        most_occur_coords = self.queue_direction[-1]
                    except:
                        pass
                    self.state.navigate(self.navigation, self.msg.found, most_occur_coords, self.power, self.rotation, gate_shape, width_height)
                    
                    """break_loop used for temp breaking of loop"""
                    #print 'press q to quit task or wait 30 secs'

                    self.counts = Counter()
                    self.queue_direction = []
                    self.last_time = time.time()

                    break_loop += 1
                #else:
                #    self.state.navigate(self.navigation, self.msg.found, coordinates, self.power, self.rotation)
                
                print 'task will stop in 600'
                print 'gate shape: {}, widthxheight: {}'.format(gate_shape, width_height)
                print 'current count: {}'.format(break_loop)
                print 'coordinates: {}'.format(coordinates)
                print '--------------------------------------------'

        # TODO will be used later when cv_controller has been completed
        # try:
        #     self.state.start(self.navigation, self.power, self.rotation)
        #     self.task_thread_start(self.state, self.tasks[self.state_num], self.navigation, self.power, self.rotation)
        # except KeyboardInterrupt:
        #     print('keyboard interrupt on cv')

        # if self.state.is_detect_done:
        #     self.state_num += 1
        #     self.state.stop()
        
        print("exit loop")

        self.foundcoord = None
        self.closePipline()
        self.navigation.cancel_h_nav()
        self.navigation.cancel_r_nav()
        self.navigation.cancel_m_nav()
        # self.state.reset()    
    # created to get most frequent coordinates from detect methods
    # once most frequent coordinates are found, sub will navigate to it
    # rather than just going to last coordinates

    # get_most_occur_coordinates ##################################################################################
    def get_most_occur_coordinates(self, last, counts):
        # if not last:
        #     most_occur = [0,0]
        for sublist in last:
            counts.update(combinations(sublist, 2))
        for key, count in counts.most_common(1):
            most_occur = key
        return most_occur

    # get_task ##################################################################################
    def get_task(self):
        self.tasks = self.config.get_config('auv', 'tasks')
        # ['gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        # 'roulette', 'pinger_a', 'cash_in']

    # start ##################################################################################
    def start(self):
        self.get_task()
        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = True
        self.navigation.start()
    
    # stop ##################################################################################
    def stop(self):
        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = False
        self.navigation.stop()