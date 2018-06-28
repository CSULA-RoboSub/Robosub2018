
class GateManeuver():
    def __init__(self):
        self.horizontal_move = {0: 'none', -1: 'left', 1: 'right'}
        self.vertical_movement = {-1: 'down', 0: 'staying', 1: 'up'}
        self.rotation_movement = {-1: 'left', 0: 'staying', 1: 'right'}
        self.move_forward = 'forward'
        self.move_backward = 'backward'
        self.start_pole = False
        self.rotation_angle = 15
        self.pole_rotation = 80
        self.depth_change = 2
        self.depth = -1
        self.sweep_timer = 20
        self.sweep_switcher = 0
        self.sweep_counter = 0
        self.sweep_direction = {0: 'right', 1: 'left'}
        self.sweep_forward = 0
        self.sweep_forward_counter = 0
        self.h_power = 100
        self.previous_direction = {0:'right', 1:'left'}
        self.strafe_rotate = {0: 'left', 1:'right'}
        self.strafe_direction = 0
        self.previous_width = 0
        self.heading_rot_change = 2
        self.heading_rot_power = 50
        self.change_m_nav_timer = 0
        self.under_gate = 0
        self.under_timer = 0

    def reset(self):
        self.sweep_switcher = 0
        self.sweep_counter = 0
        self.sweep_forward = 0
        self.sweep_forward_counter = 0
        
        self.strafe_direction = 0
        self.previous_width = 0
        self.change_m_nav_timer = 0

        self.under_gate = 0
        self.under_timer = 0

    def move_forward_method(self, navigation, coordinates, power, rotation):
        navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, self.h_power)
        navigation.r_nav(self.rotation_movement[coordinates[0]], self.rotation_angle, power)
        navigation.m_nav('power', self.move_forward, power)

    def pole(self, navigation, power):
        navigation.r_nav('right', 45, self.pole_rotation)

        navigation.cancel_r_nav()
        navigation.m_nav('power', self.move_forward, power)

        navigation.cancel_m_nav()
        navigation.r_nav('left', 90, self.pole_rotation)

        navigation.cancel_r_nav()
        navigation.m_nav('power', self.move_forward, power)

        navigation.cancel_m_nav()
        navigation.r_nav('left', 90, self.pole_rotation)

        navigation.cancel_r_nav()
        navigation.m_nav('power', self.move_forward, power)

        navigation.cancel_m_nav()
        navigation.r_nav('left', 45, self.pole_rotation)

        navigation.cancel_r_nav()
        navigation.m_nav('power', self.move_forward, power)

        navigation.cancel_m_nav()
        navigation.r_nav('right', 45, self.pole_rotation)

        navigation.cancel_r_nav()
        self.move_forward_method(navigation, power)
    
    def move_to_gate(self, navigation, coordinates, power):
        print 'move_forward_method'
        # get_rot.update_rot()
        # yaw_change = heading - get_rot.get_yaw()
        # if yaw_change < 0:
        #     navigation.r_nav('left', self.heading_rot_change, self.heading_rot_power)
        # elif yaw_change > 0:
        #     navigation.r_nav('right', self.heading_rot_change, self.heading_rot_power)
        # else:
        #     navigation.r_nav('staying', self.heading_rot_change, self.heading_rot_power)

        navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, power)
        navigation.m_nav('power', self.horizontal_move[coordinates[0]], power)
        
        '''while loop added just for 2 iterations
        so that m_nav is able to strafe before it moves forward'''
        #while self.change_m_nav_timer < 1:
        #    self.change_m_nav_timer += 1
        #self.change_m_nav_timer = 0
        navigation.cancel_m_nav()
        navigation.m_nav('power', self.move_forward, power)

    def sweep(self, navigation, power, rotation):
        # used to sweep the area in front of the sub when nothing of interest is found
        # sub will begin by turning right for 2 seconds, moving forward 2 seconds then left 2 seconds
        # the cycle will restart thereafter
        if self.sweep_forward == 0:
            navigation.r_nav(self.sweep_direction[self.sweep_switcher], rotation, 50)
            self.sweep_counter += 1
        else:
            navigation.m_nav('power', self.move_forward, power)
            self.sweep_forward_counter += 1

        if self.sweep_counter >= self.sweep_timer:
            self.sweep_switcher = 1 - self.sweep_switcher
            self.sweep_forward = 1
            self.sweep_counter = 0
        
        if self.sweep_forward_counter >= self.sweep_timer:
            self.sweep_forward = 0
            self.sweep_forward_counter = 0

    def strafe_to_square(self, navigation, power, rotation, width):
        # to change direction of strafe if previous width is larger than current
        if self.previous_width > width:
            self.strafe_direction = 1 - self.strafe_direction

        # TODO need to find test current rotation number to ensure circlar movement is not too large
        navigation.m_nav('power', self.previous_direction[self.strafe_direction], power)
        navigation.r_nav(self.strafe_rotate[self.strafe_direction], rotation, 50)
        self.previous_width = width
    
    def backup_to_square(self, navigation, power):
        # the sub may return a horizontal rectangle is the sub is too close to the gate
        # if the gate is actuallt a horizontal rectangle, need to tweak buffer in gateDetector.py
        navigation.m_nav('power', self.move_backward, power)

    def center_square(self, navigation, coordinates, power):
        # just to focus on the center of the gate to verify if sub wants to go through gate
        navigation.m_nav('power', self.horizontal_move[coordinates[0]], power)
        navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, self.h_power)

    def go_under_gate(self, navigation, coordinates, power):
        # once gate is found, along with square, and heading, sub will go towards gate
        # sub will lower height by 1 only once to help with clearance
        # there is a heading correction added, but we may not need it at the moment
        if not self.under_gate > 1:
            navigation.h_nav(self.vertical_movement[-1], self.depth_change, self.h_power)
        
        # get_rot.update_rot()
        # yaw_change = heading - get_rot.get_yaw()

        # if yaw_change < 0:
        #     navigation.r_nav('left', self.heading_rot_change, self.heading_rot_power)
        # elif yaw_change > 0:
        #     navigation.r_nav('right', self.heading_rot_change, self.heading_rot_power)
        # else:
        #     navigation.r_nav('staying', self.heading_rot_change, self.heading_rot_power)

        navigation.m_nav('power', self.move_forward, power)
        self.under_gate += 1

    def completed_gate(self):
        print 'sub has passed gate'
        #if heading is not None
        #if self.passed_gate = 1
        #return True

    def vertical(self, navigation, coordinates, power, rotation, width_height, heading):
        self.strafe_to_square(navigation, power, rotation, width_height[0])
    
    def horizontal(self, navigation, coordinates, power, rotation, width_height, heading):
        if heading is None:
            self.backup_to_square(navigation, power)
        else:
            self.go_under_gate(navigation, coordinates, power)
            self.under_timer += 1

    def square(self, navigation, coordinates, power, rotation, width_height, heading):
        #self.heading_verify_count += 1

        #if self.heading is None and self.heading_verify_count >= self.heading_verify_threshold:
        pass

    def no_shape_found(self, navigation, coordinates, power, rotation, width_height, heading):
        if heading is None:
            self.sweep(navigation, power, rotation)
        else:
            self.go_under_gate(navigation, coordinates, power)