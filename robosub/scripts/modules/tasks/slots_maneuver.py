import modules.servos.torpedo as torpedo

class SlotsManeuver():
    def __init__(self):
        ################ THRESHOLD VARIABLES ################
        self.not_found_threshold = 100
        self.found_square_threshold = 20

        ################ FLAG VARIABLES ################
        self.is_red_square_found = False
        self.is_sub_armed = True
        self.is_missle_shot = False

        ################ TIMER/COUNTER VARIABLES ################
        self.found_red_square_counter = 0

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


        ################ FRAME VARIABLES ################


    # reset ##################################################################################
    def reset(self):
        self.found_red_square_counter = 0

    def square_shape(self):
        pass

    def no_shape_found(self):
        pass

    def other_shape_found(self):
        pass

    def torpedo(self, state, side):
        self.torpedo_state[state](side)