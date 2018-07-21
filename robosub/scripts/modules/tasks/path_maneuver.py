
class PathManeuver():
    def __init__(self):
        
        ################ THRESHOLD VARIABLES ################
        self.follow_path_threshold = 200

        ################ FLAG VARIABLES ################
        self.is_moving_forward = False
        self.is_no_more_path = False
        self.is_task_complete = False
        self.is_following_path = False
        self.is_centered = False

        ################ TIMER/COUNTER VARIABLES ################
        self.follow_path_counter = 0

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

        # this is a negation of rotation movement
        # used for strafing and rotating at the same time
        self.line_up_movement = {
            -1: 'right',
             0: 'staying',
             1: 'left'
        }

        self.horizontal_move_with_forward = {
            -1: 'left',
             0: 'forward',
             1: 'right'
        }

        ################ AUV MOBILITY VARIABLES ################
        self.move_forward = 'forward'
        self.move_backward = 'backward'
        self.rotation_power = 50
        self.rotation_angle = 15
        self.r_power=100
        self.h_power=100
        self.m_power=120
        self.no_shape_m_power = 60
        
    # reset ##################################################################################
    def reset(self):
        self.is_moving_forward = False
        self.is_no_more_path = False
        self.is_task_complete = False
        self.is_following_path = False
        self.is_centered = False

        self.follow_path_counter = 0

    def no_shape_found(self, navigation, coordinates, power, rotation, width_height):
        print 'no shape found for path'
        if self.is_following_path and self.follow_path_counter >= self.follow_path_threshold:
            self.is_no_more_path = True

        navigation.m_nav('power', self.move_forward, self.no_shape_m_power)
        
    def vertical(self, navigation, coordinates, power, rotation, width_height):
        self.follow_path(navigation, coordinates ,power)
    
    def horizontal(self, navigation, coordinates, power, rotation, width_height):
        self.line_up_to_path(navigation, coordinates, power, rotation)

    def follow_path(self, navigation, coordinates, power):
        if not self.is_following_path:
            self.is_following_path = True
        navigation.m_nav('power', self.move_forward, power)
        navigation.r_nav(self.rotation_movement[coordinates[0]], self.rotation_angle, self.rotation_power)
        self.follow_path_counter += 1

    # TODO will be used when path is far away
    def move_to_path(self):
        navigation.m_nav('power', self.horizontal_move_with_forward[coordinates[0]], power)
        # navigation.r_nav(self.line_up_movement[coordinates[0]], rotation, self.rotation_power)
        # navigation.h_nav()

    #TODO must make shape as vertical as possible, to show it is lined up
    # ex. if shape changes from vertical to horizontal/square
    # sub may be facing perpendicular to the path or close to it
    def line_up_to_path(self, navigation, coordinates, power, rotation):
        navigation.m_nav('power', self.horizontal_move[coordinates[0]], power)
        navigation.r_nav(self.line_up_movement[coordinates[0]], rotation, self.rotation_power)

    def completed_path_check(self):
        check_path = self.is_no_more_path
        
        if check_path:
            self.is_task_complete = True

        return self.is_task_complete
