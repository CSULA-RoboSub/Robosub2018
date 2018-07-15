import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task
from modules.sensors.computer_vision import GateDetector
from modules.sensors.imu.gather_rotation import GetRotation

from gate_maneuver import GateManeuver

# TODO get rotation to be zero before triggering the heading verification
# add in rotation to go along with strafe

class Gate(Task):
    
    def __init__(self, Houston):
        """ To initialize Gate """
        super(Gate, self).__init__()
        
        ################ INSTANCES ################
        self.houston = Houston
        self.gate_maneuver = GateManeuver()
        self.getrotation = GetRotation()
        self.detectgate = None

        ################ THRESHOLD VARIABLES ################
        self.phase_threshold = 100
        self.heading_verify_threshold = 200
        self.under_threshold = 100
        
        ################ FLAG VARIABLES ################
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
        self.stop_task = False
        self.is_task_running = False
        self.is_complete = False

        ################ TIMER/COUNTER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0
        self.forward_counter = 0
        self.under_timer = 0
        self.passed_gate = 0
        self.heading_verify_count = 0
        self.last_time = 0
        self.counter = Counter()

        ################ DICTIONARIES ################        
        self.movement_to_square = {
            'vertical': 'right',
            'horizontal': 'backward'
        }
                                
        self.gate_phases = {
            None: self.gate_maneuver.no_shape_found,
            'vertical': self.gate_maneuver.vertical,
            'horizontal': self.gate_maneuver.horizontal,
            'square': self.gate_maneuver.square
        }

        ################ AUV MOBILITY VARIABLES ################
        self.depth_change = 1
        self.rotation_angle = 15
        self.is_heading_correct = False
        self.previous_width_height = (0,0)
        self.direction_list = []
        self.r_power=100
        self.h_power=100
        self.m_power=120   

        ################ THREAD VARIABLES ################    
        self.thread_gate = None
        self.mutex = Lock()

    # reset ##################################################################################
    def reset(self):
        self.detectgate = None

        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
        self.is_task_running = False
        self.is_complete = False

        self.not_found_timer = 0
        self.found_timer = 0
        self.forward_counter = 0
        self.under_timer = 0
        self.passed_gate = 0
        self.heading_verify_count = 0
        self.last_time = 0
        self.counter = Counter()

        self.is_heading_correct = False
        self.previous_width_height = (0,0)
        self.direction_list = []

        self.thread_gate = None
        self.stop_task = False

        self.gate_maneuver.reset()
        
    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        self.local_cvcontroller = cvcontroller
        self.is_task_running = True
        cvcontroller.start(task_name)
        self.mutex.acquire()
        count = 0
        self.last_time = time.time()
        #self.run_detect_for_task(navigation, m_power, rotation)
        while not self.stop_task:
            try:
                found, directions, gate_shape, width_height = cvcontroller.detect(task_name)
                # if directions:
                if found:
                    self.direction_list.append(directions)

                if (time.time()-self.last_time > 0.05):
                    self.last_time = time.time()
                    count += 1

                    try:
                        most_occur_coords = self.get_most_occur_coordinates(self.direction_list, self.counter)
                    except:
                        most_occur_coords = [0, 0]

                    print 'gate shape: {}, widthxheight: {}'.format(gate_shape, width_height)
                    print 'current count: {}'.format(count)
                    print 'coordinates: {}'.format(most_occur_coords)
                    print '--------------------------------------------'
                    print 'type: navigation cv 0, or task to cancel task'
                    self.navigate(navigation, found, most_occur_coords, m_power, rotation, gate_shape, width_height)
                    
                    self.counter = Counter()
                    self.direction_list = []
                    self.last_time = time.time()
            except:
                print('gate task error')

        cvcontroller.stop()     
        self.mutex.release()
        self.is_task_running = False
    # stop ##################################################################################
    def stop(self):
        # self.navigation.stop()
        # self.cvcontroller.stop()
        self.local_cvcontroller.stop()
      
    def search(self):
        pass
    
    # TODO do not need follow method anymore, will remove after testing
    # run_detect_for_task ##################################################################################
    def run_detect_for_task(self, navigation, m_power=120, rotation=15):
        self.reset_thread()

        self.thread_gate = Thread(target = self.detect, args = (navigation, m_power,rotation))
        #self.thread_gate = Thread(target=self.test)
        self.thread_gate.start()
        #self.thread_gate.join()

    # TODO do not need following method anymore, will remove after testing
    # reset_thread ##################################################################################
    def reset_thread(self):
        if self.thread_gate:
            self.thread_gate = None
            
    # detect ##################################################################################   
    def detect(self, frame):
        pass
    #     #add frame when testing complete
    #     if not self.detectgate:
    #         # self.detectgate = GateDetector.GateDetector()

    #     return self.detectgate.detect(frame)

    # def detect(self, navigation, m_power=120, rotation=15):
    #     self.last_time = time.time()
    #     self.mutex.acquire()
    #     try:
    #         print 'try detect'
    #         for i in range (0, 100):
    #             found, directions, gate_shape, width_height = self.cvcontroller.detect('gate')
    #             if found:
    #                 self.direction_list.append(directions)

    #             if (time.time()-self.last_time > 0.05):

    #                 try:
    #                     most_occur_coords = self.get_most_occur_coordinates(self.direction_list, self.counter)
    #                 except:
    #                     most_occur_coords = [0,0]

    #                 print 'gate shape: {}, widthxheight: {}'.format(gate_shape, width_height)
    #                 print 'current count: {}'.format(i)
    #                 print 'coordinates: {}'.format(most_occur_coords)
    #                 print '--------------------------------------------'
    #                 self.navigate(navigation, found, most_occur_coords, m_power, rotation, gate_shape, width_height)
                    
    #                 self.counter = Counter()
    #                 self.direction_list = []
    #                 self.last_time = time.time()

    #     finally:
    #         self.mutex.release()
    #     print()
    
    # navigate ##################################################################################
    def navigate(self, navigation, found, coordinates, power, rotation, gate_shape, width_height):
        if not self.gate_maneuver.is_moving_forward:
            navigation.cancel_r_nav()
            navigation.cancel_m_nav()
            navigation.cancel_h_nav()
        '''if self.forward_counter >= 2:
            self.is_detect_done = True'''            
        # self.gate_maneuver.sweep_forward = 0
        #TODO need to get rid of if statements and clean up code
        # if found and gate_shape == 'horizontal':
        #     self.heading_verify_count += 1
        #     if not self.is_heading_correct and self.heading_verify_count >= self.heading_verify_threshold:
        #         # self.getrotation.update_rot()
        #         self.is_heading_correct = True

        #     if not self.is_heading_correct:
        #         self.gate_maneuver.center_square(navigation, coordinates, power)

        #     else:
        #         self.gate_maneuver.move_to_gate(navigation, coordinates, power)
        # else:
        #     self.gate_phases[gate_shape](navigation, coordinates, power, rotation, width_height, self.is_heading_correct)
        self.gate_phases[gate_shape](navigation, coordinates, power, rotation, width_height, self.is_heading_correct)

                    
        self.previous_width_height = width_height

        if self.gate_maneuver.under_timer > self.under_threshold:
            self.passed_gate = 1
            print 'sub has gone under and past gate'
            
    # complete ##################################################################################
    def complete(self):
        self.is_complete = True

    # get_most_occur_coordinates ##################################################################################
    def get_most_occur_coordinates(self, direction_list, counter):
        for sublist in direction_list:
            counter.update(combinations(sublist, 2))
        for key, count in counter.most_common(1):
            most_occur = key
        return most_occur

    # TODO implement
    # bail_task ##################################################################################
    def bail_task(self):
        print 'bail gate'

    # TODO implement
    # restart_task ##################################################################################
    def restart_task(self):
        print 'restart gate'
        