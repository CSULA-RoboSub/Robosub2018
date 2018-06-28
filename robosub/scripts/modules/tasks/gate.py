from modules.sensors.computer_vision import GateDetector
from task import Task
from gate_maneuver import GateManeuver
from modules.controller.cv_controller import CVController

class Gate(Task):
    
    def __init__(self, Houston):
        """ To initialize Gate """
        super(Gate, self).__init__()
        
        self.houston = Houston
        self.gate_maneuver = GateManeuver()
        
        self.detectgate = None
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        self.not_found_timer = 0
        self.found_timer = 0
        self.gate_circle_loc = 0

        self.mState = {'off': 0,
                        'power': 1,
                        'distance': 2,
                        'front_cam_center': 3,
                        'bot_cam_center': 4,
                        'motor_time': 5}

        self.depth_change = 1
        self.phase_threshold = 50
        self.pole_rotation = 80
        self.forward_counter = 0

        self.rotation_angle = 15

        self.movement_to_square = {'vertical': 'right',
                                'horizontal': 'backward'}

        self.heading = None
        self.previous_width_height = (0,0)
        self.heading_verify_threshold = 20
        self.heading_verify_count = 0

        self.under_timer = 0
        self.under_threshold = 50

        self.passed_gate = 0

        self.gate_phases = {None: self.gate_maneuver.sweep,
                            'vertical': self.gate_maneuver.vertical,
                            'horizontal': self.gate_maneuver.horizontal,
                            'square': self.gate_maneuver.square}
    def reset(self):
        self.detectgate = None
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
        self.not_found_timer = 0
        self.found_timer = 0
        self.gate_circle_loc = 0
        self.forward_counter = 0
        self.heading = None
        self.previous_width_height = (0,0)
        self.heading_verify_count = 0
        self.under_timer = 0
        self.passed_gate = 0
        
    def detect(self, frame):
        #add frame when testing complete
        if not self.detectgate:
            self.detectgate = GateDetector.GateDetector()

        return self.detectgate.detect(frame)
    
    def navigate(self, navigation, found, coordinates, power, rotation, gate_shape, width_height):
        ''' to clear previous navigation commands'''
        navigation.cancel_r_nav()
        navigation.cancel_m_nav()
        navigation.cancel_h_nav()
        '''if self.forward_counter >= 2:
            self.is_detect_done = True'''            
        # self.gate_maneuver.sweep_forward = 0
        #TODO need to get rid of if statements and clean up code
        if found:
            #self.gate_phases[gate_shape](navigation, coordinates, power, rotation, gate_shape, width_height)
            if gate_shape == 'vertical':
                self.gate_maneuver.strafe_to_square(navigation, power, rotation, width_height[0])
                
            elif gate_shape == 'horizontal':
                if self.heading is None:
                    self.gate_maneuver.backup_to_square(navigation, power)

                else:
                    self.gate_maneuver.go_under_gate(navigation, coordinates, power)
                    self.under_timer += 1

            elif gate_shape == 'square':
                self.heading_verify_count += 1

                if self.heading == None and self.heading_verify_count >= self.heading_verify_threshold:
                    self.getrotation.update_rot()
                    self.heading = self.getrotation.get_yaw()

                if self.heading is None:
                    self.gate_maneuver.center_square(navigation, coordinates, power)

                else:
                    self.gate_maneuver.move_to_gate(navigation, coordinates, power)
        else:
            #self.gate_phases[gate_shape](navigation, power, rotation)
            self.gate_maneuver.sweep(navigation, power, rotation)

                    
        self.previous_width_height = width_height

        if self.under_timer > self.under_threshold:
            self.passed_gate = 1
    
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
        #code will be used to navigate to the pole and then circle(square)
        #around it


    def bail_task(self):
        print 'bail gate'

    def restart_task(self):
        print 'restart gate'
