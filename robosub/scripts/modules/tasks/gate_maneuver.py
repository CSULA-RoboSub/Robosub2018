
class GateManeuver():
    def __init__(self):
        self.horizontal_move = {0: 'none', -1: 'left', 1: 'right'}
        self.vertical_movement = {-1: 'down', 0: 'staying', 1: 'up'}
        self.rotation_movement = {-1: 'left', 0: 'staying', 1: 'right'}
        self.move_forward = 'forward'
        self.move_backward = 'backward'
        self.is_forward_done = False
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

    def move_forward_method(self, navigation, coordinates, power, rotation):
        navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, power)
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
    
    def move_to_gate(self, navigation, coordinates, power, get_rot, heading):
        print 'move_forward_method'
        get_rot.update_rot()
        yaw_change = heading - get_rot.get_yaw()
        if yaw_change < 0:
            navigation.r_nav('left', self.heading_rot_change, self.heading_rot_power)
        elif yaw_change > 0:
            navigation.r_nav('right', self.heading_rot_change, self.heading_rot_power)
        else:
            navigation.r_nav('staying', self.heading_rot_change, self.heading_rot_power)

        navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, power)
        navigation.m_nav('power', self.horizontal_move[coordinates[0]], power)
        
        '''while loop added just for 2 iterations
        so that m_nav is able to strafe before it moves forward'''
        while self.change_m_nav_timer < 1:
            self.change_m_nav_timer += 1
        self.change_m_nav_timer = 0
        navigation.cancel_m_nav()
        navigation.m_nav('power', self.move_forward, power)
        print 'completed move_to_gate method'

    def sweep(self, navigation, power, rotation):
        if self.sweep_forward == 0:
            navigation.r_nav(self.sweep_direction[self.sweep_switcher], rotation, 50)
            self.sweep_counter += 1
        else:
            navigation.m_nav('power', self.move_forward, power)
            self.sweep_forward_counter += 1

        ''' used to change 0 to 1 and 1 to 0 without using if statements'''
        if self.sweep_counter >= self.sweep_timer:
            self.sweep_switcher = 1 - self.sweep_switcher
            self.sweep_forward = 1
            self.sweep_counter = 0
        
        if self.sweep_forward_counter >= self.sweep_timer:
            self.sweep_forward = 0
            self.sweep_forward_counter = 0

    def strafe_to_square(self, navigation, power, rotation, width):
        print 'strafe_to_square width: {}'.format(width)
        if self.previous_width > width:
            self.strafe_direction = 1 - self.strafe_direction

        navigation.m_nav('power', self.previous_direction[self.strafe_direction], power)
        navigation.r_nav(self.strafe_rotate[self.strafe_direction], rotation, 50)
        self.previous_width = width
    
    def backup_to_square(self, navigation, power):
        print 'backup_to_square'
        navigation.m_nav('power', self.move_backward, power)

    def center_square(self, navigation, coordinates, power):
        navigation.m_nav('power', self.horizontal_move[coordinates[0]], power)
        navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, power)