from modules.sensors.computer_vision import GateDetector
from task import Task
from gate_maneuver import GateManeuver
from modules.controller.cv_controller import CVController
from modules.sensors.imu.gather_rotation import GetRotation

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

        self.mState = {'off': 0, 'power': 1, 'distance': 2, 'front_cam_center': 3, 'bot_cam_center': 4, 'motor_time': 5}

        self.depth_change = 1
        self.phase_threshold = 50
        self.pole_rotation = 80
        self.forward_counter = 0

        self.rotation_angle = 15

        self.movement_to_square = {'vertical': 'right', 'horizontal': 'backward'}

        self.heading = None
        self.previous_width_height = (0,0)
        self.get_heading_timer = 20
        self.heading_timer = 0

        self.getrotation = GetRotation()

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
        self.heading_timer = 0
        
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
        
        if found:
            self.found_timer += 1
            #self.gate_maneuver.move_to_gate(navigation, coordinates, power, rotation)

            if gate_shape == 'vertical':
                self.gate_maneuver.strafe_to_square(navigation, power, rotation, width_height[0])
            elif gate_shape == 'horizontal':
                self.gate_maneuver.backup_to_square(navigation, power)
            elif gate_shape == 'square':
                self.heading_timer += 1
                if self.heading == None and self.heading_timer >= self.get_heading_timer:
                    self.getrotation.update_rot()
                    self.heading = self.getrotation.get_yaw()
                    print 'heading: {}'.format(self.heading)
                    print 'Heading has been received and logged'

                if self.heading is None:
                    self.gate_maneuver.center_square(navigation, coordinates, power)
                else:
                    self.gate_maneuver.move_to_gate(navigation, coordinates, power, self.getrotation, self.heading)
        else:
            self.gate_maneuver.sweep(navigation, power, rotation)
                    
        self.previous_width_height = width_height

        '''elif self.found_timer > self.phase_threshold:
            if not self.gate_maneuver.start_pole:
                self.gate_maneuver.move_forward_method(navigation, power, self.getrotation, self.heading)
                self.forward_counter += 1
            else:                
                self.gate_maneuver.pole(navigation, power)
            self.found_timer = 0'''
    
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
