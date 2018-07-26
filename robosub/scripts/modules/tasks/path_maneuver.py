
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
        self.is_close_enough = False
        self.is_frame_height_max = False
        self.is_no_longer_frame_height_max = False
        self.is_initial_centered = False

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

        self.m_power_horizontal_move_with_forward = {
            -1: 70,
             0: 80,
             1: 70
        }

        self.forward_backward_move = {
            -1: 'backward',
             0: 'none',
             1: 'forward'
        }

        ################ AUV MOBILITY VARIABLES ################
        self.move_forward = 'forward'
        self.move_backward = 'backward'
        self.rotation_angle = 5
        self.r_power=80
        self.h_power=100
        self.m_power=80
        self.m_power_strafe=self.m_power_horizontal_move_with_forward[1]
        self.no_shape_m_power = 60
        self.depth_change = 0.2
        
    # reset ##################################################################################
    def reset(self):
        self.is_moving_forward = False
        self.is_no_more_path = False
        self.is_task_complete = False
        self.is_following_path = False
        self.is_centered = False
        self.is_close_enough = False
        self.is_frame_height_max = False
        self.is_no_longer_frame_height_max = False
        self.is_initial_centered = False

        self.follow_path_counter = 0

    # def no_shape_found(self, navigation, coordinates, power, rotation, width_height):
    #     print 'no shape found for path'
    #     if self.is_following_path and self.follow_path_counter >= self.follow_path_threshold:
    #         self.is_no_more_path = True

    #     navigation.m_nav('power', self.move_forward, self.no_shape_m_power)
        
    # def vertical(self, navigation, coordinates, power, rotation, width_height):
    #     self.follow_path(navigation, coordinates ,power)
    
    # def horizontal(self, navigation, coordinates, power, rotation, width_height):
    #     self.line_up_to_path(navigation, coordinates, power, rotation)

    # def follow_path(self, navigation, coordinates, power):
    #     if not self.is_following_path:
    #         self.is_following_path = True
    #     navigation.m_nav('power', self.move_forward, power)
    #     navigation.r_nav(self.rotation_movement[coordinates[0]], self.rotation_angle, self.rotation_power)
    #     self.follow_path_counter += 1

    # # TODO will be used when path is far away
    # def move_to_path(self):
    #     navigation.m_nav('power', self.horizontal_move_with_forward[coordinates[0]], power)
    #     # navigation.r_nav(self.line_up_movement[coordinates[0]], rotation, self.rotation_power)
    #     # navigation.h_nav()

    # #TODO must make shape as vertical as possible, to show it is lined up
    # # ex. if shape changes from vertical to horizontal/square
    # # sub may be facing perpendicular to the path or close to it
    # def line_up_to_path(self, navigation, coordinates, power, rotation):
    #     navigation.m_nav('power', self.horizontal_move[coordinates[0]], power)
    #     navigation.r_nav(self.line_up_movement[coordinates[0]], rotation, self.rotation_power)

    def completed_path_check(self):
        return self.is_no_longer_frame_height_max

    def center_x_or_move_forward(self, navigation, horizontal_move):
        navigation.cancel_and_m_nav('power', self.horizontal_move_with_forward[horizontal_move], self.m_power_horizontal_move_with_forward[horizontal_move])

    def center_y(self, navigation, forward_backward_move):
        navigation.cancel_and_m_nav('power', self.forward_backward_move[forward_backward_move], self.m_power)

    def dive_to_path(self, navigation):
        navigation.h_nav('down', self.depth_change, self.h_power)

    def follow_path(self, navigation, horizontal_move, rotation_direction):
        if rotation_direction == 0 and horizontal_move == 0:
            #move forward
            navigation.cancel_r_nav(self.r_power)
            self.center_x_or_move_forward(navigation, horizontal_move)
        elif rotation_direction != 0 or horizontal_move != 0:
            if rotation_direction != 0:
                navigation.cancel_and_r_nav(self.rotation_movement[rotation_direction], self.rotation_angle, self.r_power)
            else:
                navigation.cancel_r_nav(self.r_power)

            if horizontal_move != 0:
                self.center_x_or_move_forward(navigation, horizontal_move)
            else:
                navigation.cancel_m_nav(self.m_power_horizontal_move_with_forward[horizontal_move])
