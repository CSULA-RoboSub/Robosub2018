from modules.sensors.computer_vision import GateDetector
from task import Task
from gate_maneuver import GateManeuver
from modules.controller.cv_controller import CVController
from modules.sensors.imu.gather_rotation import GetRotation
from threading import Thread, Lock
import time
from collections import Counter
from itertools import combinations

class Gate(Task):
    
    def __init__(self, Houston):
        """ To initialize Gate """
        super(Gate, self).__init__()
        
        ################ INSTANCES ################
        self.houston = Houston
        self.gate_maneuver = GateManeuver()
        self.getrotation = GetRotation()
        self.cvcontroller = CVController()
        self.detectgate = None

        ################ THRESHOLD VARIABLES ################
        self.phase_threshold = 100
        self.heading_verify_threshold = 40
        self.under_threshold = 100
        
        ################ FLAG VARIABLES ################
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        ################ TIMER/COUNTER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0
        self.forward_counter = 0
        self.under_timer = 0
        self.passed_gate = 0
        self.heading_verify_count = 0
        self.last_time = 0
        self.counts = Counter()

        ################ DICTIONARIES ################
        self.mState = {'off': 0,
                        'power': 1,
                        'distance': 2,
                        'front_cam_center': 3,
                        'bot_cam_center': 4,
                        'motor_time': 5}
        
        self.movement_to_square = {'vertical': 'right',
                                'horizontal': 'backward'}
                                
        self.gate_phases = {None: self.gate_maneuver.no_shape_found,
                            'vertical': self.gate_maneuver.vertical,
                            'horizontal': self.gate_maneuver.horizontal,
                            'square': self.gate_maneuver.square}

        ################ AUV MOBILITY VARIABLES ################
        self.depth_change = 1
        self.rotation_angle = 15
        self.heading = None
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

        self.not_found_timer = 0
        self.found_timer = 0
        self.forward_counter = 0
        self.under_timer = 0
        self.passed_gate = 0
        self.heading_verify_count = 0
        self.last_time = 0
        self.counts = Counter()

        self.heading = None
        self.previous_width_height = (0,0)
        self.direction_list = []

        self.thread_gate = None

        self.gate_maneuver.reset()

    # start ##################################################################################
    def start(self, navigation, m_power=120, rotation=15):
        self.run_detect_for_task(navigation, m_power, rotation)

    # stop ##################################################################################
    def stop(self):
        #self.navigation.stop()
        pass
    
    # run_detect_for_task ##################################################################################
    def run_detect_for_task(self, navigation, m_power=120, rotation=15):
        self.reset_thread()

        self.thread_gate = Thread(target = self.detect, args = (navigation, m_power,rotation))
        #self.thread_gate = Thread(target=self.test)
        self.thread_gate.start()
        #self.thread_gate.join()

    # reset_thread ##################################################################################
    def reset_thread(self):
        if self.thread_gate:
            self.thread_gate = None

    # detect ##################################################################################   
    def detect(self, frame):
        #add frame when testing complete
        if not self.detectgate:
            self.detectgate = GateDetector.GateDetector()

        return self.detectgate.detect(frame)

    '''def detect(self, navigation, m_power=120, rotation=15):
        self.last_time = time.time()
        self.mutex.acquire()
        try:
            self.cvcontroller.start()
            found, directions, gate_shape, width_height = self.cvcontroller.detect('gate')
            self.direction_list.append(directions)
            if (time.time()-self.last_time > 0.05):
                most_occur_coords = self.get_most_occur_coordinates(self.queue_direction, self.counts)
                self.navigate(navigation, found, most_occur_coords, m_power, rotation, gate_shape, width_height)
                
                self.counts = Counter()
                self.direction_list = []
                self.last_time = time.time()

            self.cvcontroller.stop()
        finally:
            self.mutex.release()'''
    
    # navigate ##################################################################################
    def navigate(self, navigation, found, coordinates, power, rotation, gate_shape, width_height):
        navigation.cancel_r_nav()
        navigation.cancel_m_nav()
        navigation.cancel_h_nav()
        '''if self.forward_counter >= 2:
            self.is_detect_done = True'''            
        # self.gate_maneuver.sweep_forward = 0
        #TODO need to get rid of if statements and clean up code
        if found and gate_shape == 'square':
            self.heading_verify_count += 1
            if self.heading is None and self.heading_verify_count >= self.heading_verify_threshold:
                self.getrotation.update_rot()
                self.heading = self.getrotation.get_yaw()

            if self.heading is None:
                self.gate_maneuver.center_square(navigation, coordinates, power)

            else:
                self.gate_maneuver.move_to_gate(navigation, coordinates, power)
        else:
            self.gate_phases[gate_shape](navigation, coordinates, power, rotation, width_height, self.heading)

                    
        self.previous_width_height = width_height

        if self.gate_maneuver.under_timer > self.under_threshold:
            self.passed_gate = 1
            print 'sub has gone under and past gate'
        
        navigation.cancel_r_nav()
        navigation.cancel_m_nav()
        navigation.cancel_h_nav()

    # complete ##################################################################################
    def complete(self):
        pass

    # get_most_occur_coordinates ##################################################################################
    def get_most_occur_coordinates(self, last, counts):
        for sublist in last:
            counts.update(combinations(sublist, 2))
        for key, count in counts.most_common(1):
            most_occur = key
        return most_occur

    # bail_task ##################################################################################
    def bail_task(self):
        print 'bail gate'

    # restart_task ##################################################################################
    def restart_task(self):
        print 'restart gate'