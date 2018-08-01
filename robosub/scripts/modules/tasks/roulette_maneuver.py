
class RouletteManeuver():
    def __init__(self):
         ################ THRESHOLD VARIABLES ################


        ################ FLAG VARIABLES ################
        self.is_coin_aquired = False
        self.is_moving_forward = False
        self.is_coin_dropped = False
        self.is_centered = False

        ################ TIMER/COUNTER VARIABLES ################


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

        self.forward_backward_move = {
            -1: 'backward',
             0: 'staying',
             1: 'forward'
        }

        self.left_right_move = {
            -1: 'left',
             0: 'staying',
             1: 'right'
        }

        ################ AUV MOBILITY VARIABLES ################
        self.rotation_angle = 15
        self.move_forward = 'forward'
        self.move_backward = 'backward'
        self.rotation_power = 70
        self.depth_change = 2
        self.r_power=100
        self.h_power=100
        self.m_power=120
        # TODO remove soon
        self.rotation_direction = 'right'

    # reset ##################################################################################
    def reset(self):
        self.is_moving_forward = False
        self.is_centered = False

    # go_over_black ##################################################################################
    def go_over_black(self, navigation, found, most_occur_coords, m_power, rotation, shape):
        print 'testing go over black'
        print self.coord_boxes[(most_occur_coords[0], most_occur_coords[1])]

    # go_over_red ##################################################################################
    def go_over_red(self):
        print 'testing go over red'

    # go_over_green ##################################################################################
    def go_over_green(self):
        print 'testing go over green'
    
    # completed_roulette_check ##################################################################################
    def completed_roulette_check(self):
        
        return False

    def center_color(self, navigation, coordinates, power):
        if coordinates[0] != 0:
            navigation.cancel_and_m_nav('power', self.left_right_move[coordinates[0]], power)
        elif coordinates[1] != 0:
            navigation.cancel_and_m_nav('power', self.forward_backward_move[coordinates[1]], power)
        else:
            navigation.cancel_m_nav()

        self.nothing_found_counter = 0