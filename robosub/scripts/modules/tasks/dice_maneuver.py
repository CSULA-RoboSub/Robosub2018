
class DiceManeuver():
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

        ################ AUV MOBILITY VARIABLES ################
        self.move_forward = 'forward'
        self.move_backward = 'backward'

    # reset ##################################################################################
    def reset(self):
        pass