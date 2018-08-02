import modules.servos.torpedo as torpedo

class SlotsManeuver():
    def __init__(self):
        ################ THRESHOLD VARIABLES ################
        self.not_found_threshold = 100
        self.found_square_threshold = 20
        self.heading_verify_threshold = 40
        self.line_up_for_shot_threshold = 20

        ################ FLAG VARIABLES ################
        self.is_heading_correct = False
        self.is_red_square_found = False
        self.is_rotated_to_center = False
        self.is_lined_up_for_torpedo = False
        self.is_torpedo_primed = True
        self.is_torpedo_fired = False
        self.is_task_complete = False

        ################ TIMER/COUNTER VARIABLES ################
        self.found_red_square_counter = 0
        self.nothing_found_counter = 0
        self.heading_verify_count = 0
        self.line_up_for_shot_counter = 0

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

        self.horizontal_move_with_forward = {
            -1: 'left',
             0: 'forward',
             1: 'right'
        }

        self.torpedo_state = {
            'prime': torpedo.prime_torpedo,
            'fire': torpedo.fire_torpedo
        }

        self.torpedo_sides = {
            0: 'left',
            1: 'right'
        }

        ################ CONSTANTS ################
        self.prime = 'prime'
        self.fire = 'fire'
        self.left = 'left'
        self.right = 'right'

        ################ AUV MOBILITY VARIABLES ################
        self.move_forward = 'forward'
        self.rotation_direction = 'right'
        self.rotation_angle = 5
        self.rotation_power = 70
        self.h_power = 100

        ################ FRAME VARIABLES ################
        self.frame_area = 744*480
        self.max_red_square_area = (self.frame_area / 3) * 2


    # reset ##################################################################################
    def reset(self):
        self.found_red_square_counter = 0
        self.nothing_found_counter = 0
        self.heading_verify_count = 0
        self.line_up_for_shot_counter = 0

        self.is_heading_correct = False
        self.is_red_square_found = False
        self.is_rotated_to_center = False
        self.is_lined_up_for_torpedo = False
        self.is_torpedo_primed = False
        self.is_torpedo_fired = False
        self.is_task_complete = False

    def offset_movement_after_square(self, navigation):
        pass

    # will be used for priming and firing of torpedos
    def torpedo(self, state, side):
        self.torpedo_state[state](side)

    def completed_slots_check(self):
        check_head = self.is_heading_correct
        check_torpedo = self.is_torpedo_fired

        if check_head and check_torpedo:
            self.is_task_complete = True

        return self.is_task_complete

    def rotate(self, navigation, power, rotation):
        self.is_rotated_to_center = False
        navigation.cancel_and_r_nav(self.rotation_direction, self.rotation_angle, self.rotation_power)

    def square(self, navigation, found, coordinates, power, rotation, width_height):
        print 'square'
        self.torpedo_check()
        
        self.heading_verify_count += 1
        if not self.is_heading_correct and self.heading_verify_count >= self.heading_verify_threshold and coordinates[0] == 0 and coordinates[1] == 0 and found:
            self.is_heading_correct = True

        if not self.is_heading_correct:
            self.center_red_square(navigation, coordinates, power)
        else:
            self.move_towards_red_square(navigation, coordinates, power, width_height)


    def no_shape_found(self, navigation, found, coordinates, power, rotation, width_height):
        print 'no_shape_found'
        self.torpedo_check()
        # self.square(navigation, found, coordinates, power, rotation, width_height)
        # self.vertical(navigation, found, coordinates, power, rotation, width_height)
        # self.horizontal(navigation, found, coordinates, power, rotation, width_height)
        # self.center_red_square(navigation, coordinates, power)
        # self.move_towards_red_square(navigation, coordinates, power, width_height)
        # self.offset_movement_after_square(navigation)
        # self.rotate(navigation, power, rotation)


    def vertical(self, navigation, found, coordinates, power, rotation, width_height):
        print 'vertical'
        self.torpedo_check()

    def horizontal(self, navigation, found, coordinates, power, rotation, width_height):
        print 'horizontal'
        self.torpedo_check()

    def center_red_square(self, navigation, coordinates, power):
        self.nothing_found_counter = 0
        if not coordinates[0] == 0:
            navigation.cancel_and_m_nav('power', self.horizontal_move[coordinates[0]], power)
        else:
            navigation.cancel_m_nav()

        if not coordinates[1] == 0:
            navigation.cancel_and_h_nav(self.vertical_movement[coordinates[1]], self.depth_change, self.h_power)
        else:
            navigation.cancel_h_nav()

    def move_towards_red_square(self, navigation, coordinates, power, width_height):
        self.nothing_found_counter = 0
        w_h_area = width_height[0] * width_height[1]
        
        if w_h_area < self.max_red_square_area and not self.is_lined_up_for_torpedo:
            navigation.cancel_r_nav()
            navigation.cancel_and_m_nav('power', self.move_forward, power)
        else:
            print 'sub is in ideal shooting position'
            self.is_lined_up_for_torpedo = True
            # TODO move to offset to line up torpedo for shot
            self.offset_movement_after_square(navigation)
            navigation.cancel_m_nav()

    def torpedo_check(self):
        if self.is_lined_up_for_torpedo:
            self.torpedo(self.prime, self.right)
            self.torpedo(self.prime, self.left)
            self.is_torpedo_primed = True
            return
        if self.is_torpedo_primed:
            self.torpedo(self.fire, self.right)
            self.torpedo(self.fire, self.left)
            self.is_torpedo_fired = True
            return
