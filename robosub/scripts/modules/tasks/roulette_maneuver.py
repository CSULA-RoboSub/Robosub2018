
class RouletteManeuver():
    def __init__(self):
         ################ THRESHOLD VARIABLES ################


        ################ FLAG VARIABLES ################


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
        pass

    # go_over_black ##################################################################################
    def go_over_black(self):
        print 'testing go over black'

    # go_over_red ##################################################################################
    def go_over_red(self):
        pass

    # go_over_green ##################################################################################
    def go_over_green(self):
        pass
    
    # completed_roulette_check ##################################################################################
    def completed_roulette_check(self):
        
        return False