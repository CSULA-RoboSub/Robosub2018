
class DiceManeuver():
    def __init__(self):
        ################ THRESHOLD VARIABLES ################


        ################ FLAG VARIABLES ################
        self.is_moving_forward = False

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
        self.is_moving_forward = False
    
    def touch_die(self):
        pass

    def back_up_from_die(self):
        pass

    def find_die(self):
        pass

    def level_to_die(self):
        pass