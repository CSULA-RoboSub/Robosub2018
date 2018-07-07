
class GateManeuver():
    def __init__(self):

        ################ THRESHOLD VARIABLES ################
        self.sweep_timer = 60
        self.nothing_found_threashold = 100
        self.heading_verify_threshold = 200

        ################ FLAG VARIABLES ################
        self.is_heading_correct = False

        ################ TIMER/COUNTER VARIABLES ################
        self.sweep_forward_counter = 0
        self.sweep_switcher = 0
        self.sweep_counter = 0
        self.change_m_nav_timer = 0
        self.under_gate = 0
        self.under_timer = 0
        self.strafe_direction = 0
        self.nothing_found_counter = 0
        self.heading_verify_count = 0

        ################ DICTIONARIES ################
        self.horizontal_move = {-1: 'left',
                                0: 'none',
                                1: 'right'}

        self.vertical_movement = {-1: 'down',
                                0: 'staying',
                                1: 'up'}
                                
        self.rotation_movement = {-1: 'left',
                                0: 'staying',
                                1: 'right'}

        self.previous_direction = {0:'right',
                                1:'left'}

        self.strafe_rotate = {0: 'left',
                                1:'right'}

        self.sweep_rotation = {0:45,
                                1:90}

        self.sweep_direction = {0: self.sweep_right,
                                1: self.sweep_left,
                                2: self.sweep_right,
                                3: self.sweep_forward}

        self.move_forward = 'forward'
        self.move_backward = 'backward'

        # self.sweep_direction = {0: 'right',
                                # 1: 'left'}

        ################ AUV MOBILITY VARIABLES ################
        self.rotation_angle = 15
        self.depth_change = 1
        self.depth = -1
        self.h_power = 100
        self.rotation_direction = 'right'
        self.rotation_power = 70
        self.previous_width = 0
        # self.heading_rot_change = 2
        # self.heading_rot_power = 50
        self.is_moving_forward = False
        self.sweep_power = 80
        self.sweep_right_rotation_power = 60
        self.sweep_right_angle = 45
        self.sweep_left_rotation_power = 80
        self.sweep_left_angle = 90

    def reset(self):
        self.sweep_switcher = 0
        self.sweep_counter = 0
        self.sweep_forward_counter = 0
        
        self.strafe_direction = 0
        self.previous_width = 0
        self.change_m_nav_timer = 0

        self.under_gate = 0
        self.under_timer = 0
        self.is_moving_forward = False

        self.nothing_found_counter = 0

    def move_forward_method(self, navigation, coordinates, power, rotation):
        navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, self.h_power)
        navigation.r_nav(self.rotation_movement[coordinates[0]], self.rotation_angle, power)
        navigation.m_nav('power', self.move_forward, power)
    
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

        # navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, power)
        # navigation.m_nav('power', self.horizontal_move[coordinates[0]], power)
        
        if not self.is_moving_forward:
            self.is_moving_forward = True
            navigation.m_nav('power', self.move_forward, power)
            navigation.h_nav('down', self.depth_change, self.h_power)

    def sweep(self, navigation, power, rotation):
        # used to sweep the area in front of the sub when nothing of interest is found
        # sub will begin by turning right for 2 seconds, moving forward 2 seconds then left 2 seconds
        # the cycle will restart thereafter
        '''if self.sweep_forward == 0:
            navigation.r_nav(self.sweep_direction[self.sweep_switcher], self.sweep_rotation[self.sweep_switcher], 50)
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
            self.sweep_forward_counter = 0'''

        self.sweep_direction[self.sweep_switcher](navigation, power, rotation)
        self.sweep_counter += 1
        if self.sweep_counter >= self.sweep_timer:
            self.sweep_switcher += 1
            self.sweep_counter = 0
            if self.sweep_switcher >= 4:
                self.sweep_switcher = 0

    def sweep_right(self, navigation, power, rotation):
        # navigation.r_nav(self.sweep_direction[self.sweep_switcher], self.sweep_rotation[self.sweep_switcher], 50)
        navigation.r_nav('right', self.sweep_right_angle, self.sweep_right_rotation_power)
    
    def sweep_left(self, navigation, power, rotation):
        # navigation.r_nav(self.sweep_direction[self.sweep_switcher], self.sweep_rotation[self.sweep_switcher], 50)
        navigation.r_nav('left', self.sweep_left_angle, self.sweep_left_rotation_power)

    def sweep_forward(self, navigation, power, rotation):
        navigation.m_nav('power', self.move_forward, power)

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
        self.nothing_found_counter = 0
        navigation.m_nav('power', self.horizontal_move[coordinates[0]], power)

        if not coordinates[1] == 0:
            navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, self.h_power)

    def rotate(self, navigation, power, rotation):
        navigation.r_nav(self.rotation_direction, rotation, power)
        

    def completed_gate(self):
        print 'sub has passed gate'
        #if heading is not None
        #if self.passed_gate = 1
        #return True

    def vertical(self, navigation, coordinates, power, rotation, width_height, is_heading_correct):
        if not self.is_heading_correct:
            # self.strafe_to_square(navigation, power, rotation, width_height[0])
            self.strafe_to_square(navigation, 50, 50, width_height[0])
        else:
            self.move_to_gate(navigation, coordinates, power)
            self.under_timer += 1
        # print 'performing vertical'
    
    def horizontal(self, navigation, coordinates, power, rotation, width_height, is_heading_correct):
        # if not is_heading_correct:
        #     self.backup_to_square(navigation, power)
        # else:
        #     self.move_to_gate(navigation, coordinates, power)
        #     self.under_timer += 1
        self.heading_verify_count += 1
        if not self.is_heading_correct and self.heading_verify_count >= self.heading_verify_threshold:
            # self.getrotation.update_rot()
            self.is_heading_correct = True

        if not self.is_heading_correct:
            self.center_square(navigation, coordinates, power)

        else:
            self.move_to_gate(navigation, coordinates, power)
        # print 'performing horizontal'

        
    def square(self, navigation, coordinates, power, rotation, width_height, is_heading_correct):
        #self.heading_verify_count += 1

        #if self.heading is None and self.heading_verify_count >= self.heading_verify_threshold:

        self.heading_verify_count += 1
        if not self.is_heading_correct and self.heading_verify_count >= self.heading_verify_threshold:
            # self.getrotation.update_rot()
            self.is_heading_correct = True

        if not self.is_heading_correct:
            self.center_square(navigation, coordinates, power)

        else:
            self.move_to_gate(navigation, coordinates, power)

        # print 'performing square'

    def no_shape_found(self, navigation, coordinates, power, rotation, width_height, is_heading_correct):
        if not self.is_heading_correct:
            if self.nothing_found_counter >= self.nothing_found_threashold:
                self.rotate(navigation, self.rotation_power, rotation)
                #self.sweep(navigation, self.sweep_power, rotation)
            self.nothing_found_counter += 1
        else:
            self.move_to_gate(navigation, coordinates, power)
        # print 'performing no shape'