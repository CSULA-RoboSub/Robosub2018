
class GateManeuver():
    def __init__(self):

        ################ THRESHOLD VARIABLES ################
        self.sweep_timer = 60
        self.nothing_found_threashold = 100
        self.heading_verify_threshold = 80
        self.forward_movement_threshold = 400

        ################ FLAG VARIABLES ################
        self.is_heading_correct = False
        self.is_past_gate = False
        self.is_moving_forward = False
        self.is_task_complete = False
        self.rotated_to_center = False
        self.is_strafed_to_square = False

        ################ TIMER/COUNTER VARIABLES ################
        self.sweep_forward_counter = 0
        self.sweep_switcher = 0
        self.sweep_counter = 0
        self.change_m_nav_timer = 0
        self.forward_movement_timer = 0
        self.strafe_direction = 0
        self.nothing_found_counter = 0
        self.heading_verify_count = 0

        ################ DICTIONARIES ################
        self.horizontal_move = {
            -1: 'left',
             0: 'none',
             1: 'right'
        }

        self.vertical_movement = {
            -1: 'down',
             0: 'staying',
             1: 'up'
        }
                                
        self.rotation_movement = {
            -1: 'left',
             0: 'staying',
             1: 'right'
        }

        self.previous_direction = {
            0:'right',
            1:'left'
        }

        self.strafe_rotate = {
            0: 'left',
            1:'right'
        }

        self.sweep_rotation = {
            0:45,
            1:90
        }

        self.sweep_direction = {
            0: self.sweep_right,
            1: self.sweep_left,
            2: self.sweep_right,
            3: self.sweep_forward
        }

        ################ AUV MOBILITY VARIABLES ################
        self.rotation_angle = 10
        self.depth_change = 2
        self.depth = -1
        self.h_power = 100
        self.move_forward = 'forward'
        self.move_backward = 'backward'
        self.rotation_direction = 'right'
        self.rotation_power = 70
        self.previous_width = 0
        # self.heading_rot_change = 2
        # self.heading_rot_power = 50
        self.sweep_power = 70
        self.sweep_right_rotation_power = 60
        self.sweep_right_angle = 45
        self.sweep_left_rotation_power = 80
        self.sweep_left_angle = 90

    # reset ##################################################################################
    def reset(self):
        self.sweep_switcher = 0
        self.sweep_counter = 0
        self.sweep_forward_counter = 0
        
        self.strafe_direction = 0
        self.previous_width = 0
        self.change_m_nav_timer = 0

        self.forward_movement_timer = 0

        self.is_moving_forward = False
        self.is_heading_correct = False
        self.is_past_gate = False
        self.is_task_complete = False
        self.rotated_to_center = False
        self.is_strafed_to_square = False

        self.nothing_found_counter = 0
        self.heading_verify_count = 0
    
    # move_to_gate ##################################################################################
    def move_to_gate(self, navigation, coordinates, power):
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
        self.nothing_found_counter = 0
        self.forward_movement_timer += 1
        if not self.is_moving_forward:
            self.is_moving_forward = True

            navigation.cancel_r_nav()
            navigation.cancel_and_m_nav('power', self.move_forward, power)
            navigation.cancel_and_h_nav('down', self.depth_change, self.h_power)

            #swap camera to bottom facing

        if self.forward_movement_timer >= self.forward_movement_threshold:
            self.is_past_gate = True
            
    # sweep ##################################################################################
    def sweep(self, navigation, power, rotation):
        self.sweep_direction[self.sweep_switcher](navigation, power, rotation)
        self.sweep_counter += 1
        if self.sweep_counter >= self.sweep_timer:
            self.sweep_switcher += 1
            self.sweep_counter = 0
            if self.sweep_switcher >= 4:
                self.sweep_switcher = 0

    # sweep_right ##################################################################################
    def sweep_right(self, navigation, power, rotation):
        # navigation.r_nav(self.sweep_direction[self.sweep_switcher], self.sweep_rotation[self.sweep_switcher], 50)
        navigation.cancel_and_r_nav('right', self.sweep_right_angle, self.sweep_right_rotation_power)
    
    # sweep_left ##################################################################################
    def sweep_left(self, navigation, power, rotation):
        # navigation.r_nav(self.sweep_direction[self.sweep_switcher], self.sweep_rotation[self.sweep_switcher], 50)
        navigation.cancel_and_r_nav('left', self.sweep_left_angle, self.sweep_left_rotation_power)

    # sweep_forward ##################################################################################
    def sweep_forward(self, navigation, power, rotation):
        navigation.cancel_and_m_nav('power', self.move_forward, power)

    # strafe_to_square ##################################################################################
    def strafe_to_square(self, navigation, power, rotation, width):
        # to change direction of strafe if previous width is larger than current
        self.nothing_found_counter = 0
        if self.previous_width > width:
            self.strafe_direction = 1 - self.strafe_direction

        # TODO need to find test current rotation number to ensure circlar movement is not too large---------------
        navigation.cancel_and_m_nav('power', self.previous_direction[self.strafe_direction], power)
        # navigation.cancel_and_r_nav(self.strafe_rotate[self.strafe_direction], rotation, 2)   
        self.previous_width = width
        self.is_strafed_to_square = True
    
    # backup_to_square ##################################################################################
    def backup_to_square(self, navigation, power):
        # the sub may return a horizontal rectangle is the sub is too close to the gate
        # if the gate is actuallt a horizontal rectangle, need to tweak buffer in gateDetector.py
        navigation.cancel_and_m_nav('power', self.move_backward, power)

    # center_square ##################################################################################
    def center_square(self, navigation, coordinates, power):
        # just to focus on the center of the gate to verify if sub wants to go through gate
        self.nothing_found_counter = 0
        if not coordinates[0] == 0:
            navigation.cancel_and_m_nav('power', self.horizontal_move[coordinates[0]], power)
        else:
            navigation.cancel_m_nav()

        if not coordinates[1] == 0:
            navigation.cancel_and_h_nav(self.vertical_movement[coordinates[1]], self.depth_change, self.h_power)
        else:
            navigation.cancel_h_nav()

    # rotate ##################################################################################
    def rotate(self, navigation, power, rotation):
        self.rotated_to_center = False
        navigation.cancel_and_r_nav(self.rotation_direction, self.rotation_angle, self.rotation_power)
        
    # completed_gate ##################################################################################
    def completed_gate_check(self):
        #if heading is not None
        #if self.passed_gate = 1
        #return True
        check_head = self.is_heading_correct
        check_forward = self.is_moving_forward
        check_gate = self.is_past_gate

        if check_head and check_forward and check_gate:
            self.is_task_complete = True

        return self.is_task_complete

    # vertical ##################################################################################
    def vertical(self, navigation, coordinates, power, rotation, width_height, found):
        if not self.is_heading_correct:
            # self.strafe_to_square(navigation, power, rotation, width_height[0])
            self.strafe_to_square(navigation, 100, 50, width_height[0])
        else:
            self.move_to_gate(navigation, coordinates, power)
        # print 'performing vertical'
    
    # horizontal ##################################################################################
    def horizontal(self, navigation, coordinates, power, rotation, width_height, found):
        # if not is_heading_correct:
        #     self.backup_to_square(navigation, power)
        # else:
        #     self.move_to_gate(navigation, coordinates, power)
        #     self.forward_movement_timer += 1
        if self.is_strafed_to_square:
            self.rotated_to_center = False
            self.is_strafed_to_square = False
            return

        self.heading_verify_count += 1
        if not self.is_heading_correct and self.heading_verify_count >= self.heading_verify_threshold and coordinates[0] == 0 and coordinates[1] == 0 and found:
            # self.getrotation.update_rot()
            self.is_heading_correct = True

        if not self.is_heading_correct:
            self.center_square(navigation, coordinates, power)

        else:
            self.move_to_gate(navigation, coordinates, power)
        # print 'performing horizontal'

    # square ##################################################################################    
    def square(self, navigation, coordinates, power, rotation, width_height, found):
        #self.heading_verify_count += 1

        #if self.heading is None and self.heading_verify_count >= self.heading_verify_threshold:
        if self.is_strafed_to_square:
            self.rotated_to_center = False
            self.is_strafed_to_square = False
            return

        self.heading_verify_count += 1
        if not self.is_heading_correct and self.heading_verify_count >= self.heading_verify_threshold and coordinates[0] == 0 and coordinates[1] == 0 and found:
            # self.getrotation.update_rot()
            self.is_heading_correct = True

        if not self.is_heading_correct:
            self.center_square(navigation, coordinates, power)

        else:
            self.move_to_gate(navigation, coordinates, power)

        # print 'performing square'

    # no_shape_found ##################################################################################
    def no_shape_found(self, navigation, coordinates, power, rotation, width_height, found):
        if not self.is_heading_correct:
            if self.nothing_found_counter >= self.nothing_found_threashold:
                self.rotate(navigation, self.rotation_power, rotation)
                #self.sweep(navigation, self.sweep_power, rotation)
                # pass
            self.nothing_found_counter += 1
        else:
            self.move_to_gate(navigation, coordinates, power)
            # TODO need to add movement after gate here to search for path
            # using if statement with self.is_passed_gate
        # print 'performing no shape'
        
    # rotate_to_center ##################################################################################
    def rotate_to_center(self, navigation, coordinates):
        if coordinates[0] != 0:
            navigation.cancel_and_r_nav(self.rotation_movement[coordinates[0]], self.rotation_angle, self.rotation_power)
        else:
            navigation.cancel_r_nav()

        self.nothing_found_counter = 0

    # movement_after_gate ##################################################################################
    def movement_after_gate(self):
        pass

    # plan_b_movement ##################################################################################
    def plan_b_movement(self, navigation, coordinates, power, rotation, width_height, found):
        navigation.cancel_and_m_nav('power', self.move_forward, power)