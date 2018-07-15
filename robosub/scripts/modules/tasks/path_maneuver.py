
class PathManeuver():
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

    def no_shape_found(self, navigation, coordinates, power, rotation, width_height):
        pass

    def vertical(self, navigation, coordinates, power, rotation, width_height):
        self.follow_path(navigation, coordinates ,power)
    
    def horizontal(self, navigation, coordinates, power, rotation, width_height):
        pass

    def follow_path(self, navigation, coordinates, power):
        navigation.m_nav('power', self.move_forward, power)

    def move_to_path(self):
        pass