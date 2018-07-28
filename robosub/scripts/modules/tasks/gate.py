import time

from threading import Thread, Lock
from collections import Counter
from itertools import combinations

from task import Task
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
        # self.heading_verify_threshold = 100
        self.under_threshold = 100
        self.cant_find_threshold = 2000
        self.is_moving_forward_camera_changed_threshold = 100
        self.rotated_to_center_verify_threshold = 20 
        
        ################ FLAG VARIABLES ################
        self.is_found = False
        self.stop_task = False
        # self.is_complete = False
        self.is_camera_changed = False
        self.is_moving_forward_camera_changed = False

        ################ TIMER/COUNTER VARIABLES ################
        self.not_found_timer = 0
        self.found_timer = 0
        self.forward_counter = 0
        self.under_timer = 0
        self.passed_gate = 0
        self.heading_verify_count = 0
        self.last_time = 0
        self.counter = Counter()
        self.cant_find_counter = 0
        self.rotated_to_center_verify = 0 

        ################ DICTIONARIES ###########################        
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

        ################ CONSTANTS ###########################  
        self.path_task_name = 'path'
        self.path_camera_direction = 'down'
        
        ################ AUV MOBILITY VARIABLES ################
        self.depth_change = 1
        self.rotation_angle = 5
        self.is_heading_correct = False
        self.previous_width_height = (0,0)
        self.direction_list = []
        self.r_power=70
        self.h_power=100
        self.m_power=120   
        self.found = False
        # self.orientation_heading = None
        
        ################ THREAD VARIABLES ################    
        self.thread_gate = None
        self.mutex = Lock()

    # reset ##################################################################################
    def reset(self):
        self.detectgate = None

        self.is_found = False
        # self.is_complete = False
        self.is_camera_changed = False
        self.is_moving_forward_camera_changed = False

        self.not_found_timer = 0
        self.found_timer = 0
        self.forward_counter = 0
        self.under_timer = 0
        self.passed_gate = 0
        self.heading_verify_count = 0
        self.last_time = 0
        self.counter = Counter()
        self.rotated_to_center_verify = 0 
        # self.is_moving_forward_camera_changed_counter = 0

        self.is_heading_correct = False
        self.found = False
        # self.orientation_heading = None
        self.previous_width_height = (0,0)
        self.direction_list = []

        self.thread_gate = None
        self.stop_task = False

        self.gate_maneuver.reset()
        
    # start ##################################################################################
    def start(self, task_name, navigation, cvcontroller, m_power=120, rotation=15):
        # self.orientation_heading = navigation.waypoint.get_dvl_yaw()
        self.local_cvcontroller = cvcontroller
        cvcontroller.camera_direction = 'forward'  
        cvcontroller.start(task_name)
        self.mutex.acquire()
        count = 0
        self.last_time = time.time()
        while not self.stop_task and not self.complete():
            navigation.do_depth_cap(self.h_power)
            
            if not self.gate_maneuver.is_moving_forward:
                # try:
                found, directions, gate_shape, width_height = cvcontroller.detect(task_name)
                # if directions:
                if found:
                    self.direction_list.append(directions)

                if (time.time()-self.last_time > 0.05):
                    count += 1

                    try:
                        most_occur_coords = self.get_most_occur_coordinates(self.direction_list, self.counter)
                    except:
                        most_occur_coords = [0, 0]

                    print 'running {} task'.format(task_name)
                    print 'gate shape: {}, widthxheight: {}'.format(gate_shape, width_height)
                    print 'current count: {}'.format(count)
                    print 'coordinates: {}'.format(most_occur_coords)
                    print '--------------------------------------------'
                    print 'type: navigation cv 0, or task to cancel task'
                    self.navigate(navigation, found, most_occur_coords, m_power, rotation, gate_shape, width_height)
                    
                    self.last_time = time.time()
                    self.counter = Counter()
                    self.direction_list = []
                # except:
                #     print('gate task error')

            elif not self.is_moving_forward_camera_changed:
                self.is_moving_forward_camera_changed = True
                # self.is_moving_forward_camera_changed_counter = 0
                count = 0
                cvcontroller.change_camera_to(self.path_camera_direction, self.path_task_name)

            elif self.is_moving_forward_camera_changed and count < self.is_moving_forward_camera_changed_threshold:
                found, directions, gate_shape, width_height = cvcontroller.detect(self.path_task_name)
                if (time.time()-self.last_time > 0.05):
                    print 'counter since camera change: {}'.format(count)
                    count += 1
                    self.last_time = time.time()

                    # self.is_moving_forward_camera_changed_counter += 1

            elif self.is_moving_forward_camera_changed and count >= self.is_moving_forward_camera_changed_threshold:
                found, directions, gate_shape, width_height = cvcontroller.detect(self.path_task_name)
                if (time.time()-self.last_time > 0.05):
                    print 'counter since camera change: {}'.format(count)     
                    count += 1
                    self.last_time = time.time()
                    if found and gate_shape:
                        navigation.cancel_all_nav()
                        
                        self.gate_maneuver.is_past_gate = True

            else:
                print 'logic error in gate.py start'

        # TODO we can implement a plan_b from gate_maneuver if detection does not work
        # will need to keep heading from orientation
        cvcontroller.stop()     

        self.mutex.release()

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
    
    # navigate ##################################################################################
    def navigate(self, navigation, found, coordinates, power, rotation, gate_shape, width_height):
        if not self.gate_maneuver.rotated_to_center and gate_shape:
            if coordinates[0] == 0:
                self.rotated_to_center_verify += 1
            
            elif coordinates[0] != 0:
                self.rotated_to_center_verify = 0    

            #-------------------------------------------------------------------
            if coordinates[0] == 0 and self.rotated_to_center_verify >= self.rotated_to_center_verify_threshold:  
                self.gate_maneuver.rotated_to_center = True

            else:
                self.gate_maneuver.rotate_to_center(navigation, coordinates)
        else:
            self.gate_phases[gate_shape](navigation, coordinates, power, rotation, width_height, found)

                    
        self.previous_width_height = width_height

        # if self.gate_maneuver.under_timer > self.under_threshold:
        #     self.passed_gate = 1
        #     print 'sub has gone under and past gate'
            
    # complete ##################################################################################
    def complete(self):
        # if self.gate_maneuver.completed_gate() and self.is_camera_changed and self.found:
        #     self.is_complete = True
        return self.gate_maneuver.completed_gate_check()

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
        