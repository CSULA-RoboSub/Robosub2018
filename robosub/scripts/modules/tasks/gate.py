from modules.sensors.computer_vision import GateDetector
from task import Task
from modules.controller.cv_controller import CVController

class Gate(Task):
    
    def __init__(self, Houston):
        """ To initialize Gate """
        super(Gate, self).__init__()
        
        self.houston = Houston
        
        self.detectgate = None
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        self.rotation_direction = 'right'

        self.not_found_timer = 0
        self.found_timer = 0
        self.gate_circle_loc = 0

        self.mState = {'off': 0, 'power': 1, 'distance': 2, 'front_cam_center': 3, 'bot_cam_center': 4, 'motor_time': 5}
        self.horizontal_move = {0: 'none', -1: 'left', 1: 'right'}
        self.vertical_movement = {-1: 'down', 0: 'staying', 1: 'up'}

        self.move_forward = 'forward'
        self.depth_change = 5

        self.depth = -1
        self.rotation_direction = 'right'
    
    def detect(self, frame):
        #add frame when testing complete
        if not self.detectgate:
            self.detectgate = GateDetector.GateDetector()

        found, gate_coordinates = self.detectgate.detect(frame)
        return found, gate_coordinates
    
    def navigate(self, navigation, found, coordinates, power, rotation):
        if found:
            if coordinates == [0,0]:
                navigation.m_nav('power', 'none', 0)
                navigation.m_nav('power', self.move_forward, power)
            else:
                navigation.m_nav('power', 'none', 0)
                navigation.m_nav('power', self.horizontal_move[coordinates[0]], power)

                navigation.h_nav('staying', 0, 0)
                navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, power)
        else:
            navigation.r_nav('staying', 0, 0)
            navigation.r_nav(self.rotation_direction, rotation, power)
        
        navigation.ros_sleep(0.01)
    
    def complete(self):
        #code below is not needed anymore
        '''if (self.gate_circle_loc < 2*math.pi):
            self.gate_circle_loc += math.pi/100
            x = math.sin(self.gate_circle_loc)
            y = math.cos(self.gate_circle_loc)
            lower_bound = -.33
            upper_bound = .33

            if (x >= upper_bound):
                coord_x = 1
            elif (x < upper_bound and x >= lower_bound):
                coord_x = 0
            elif (x < lower_bound):
                coord_x = -1

            if (y >= upper_bound):
                coord_y = 1
            elif (y < upper_bound and y >= lower_bound):
                coord_y = 0
            elif (y < lower_bound):
                coord_y = -1
        else:
            self.is_gate_done = True
            print('circling gate completed')
            coord_x = 0
            coord_y = 0
        
        return True, [coord_x, coord_y]
        '''
        pass

    def bail_task(self):
        print 'bail gate'

    def restart_task(self):
        print 'restart gate'
