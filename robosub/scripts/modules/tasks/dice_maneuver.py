
class DiceManeuver():
    def __init__(self):
        ################ THRESHOLD VARIABLES ################
        self.touching_die_threshold = 100

        ################ FLAG VARIABLES ################
        self.is_moving_forward = False
        self.is_rotated_to_center = False
        self.is_1st_die_touched = False
        self.is_2nd_die_touched = False
        self.is_task_complete = False

        ################ TIMER/COUNTER VARIABLES ################
        self.touching_die_counter = 0


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
        self.h_power = 100
        # TODO remove soon
        self.rotation_direction = 'right'

        ################ FRAME VARIABLES ################
        # self.frame = (744, 480)
        self.frame = (640, 480)

    # reset ##################################################################################
    def reset(self):
        self.is_moving_forward = False
        self.is_rotated_to_center = False
        self.is_1st_die_touched = False
        self.is_2nd_die_touched = False
        self.is_task_complete = False
    
    # touch_die ##################################################################################
    def touch_die(self, navigation, coordinates, power, rotation, width_height):
        navigation.m_nav('power', self.horizontal_move_with_forward[coordinates[0]], power)
        navigation.r_nav(self.rotation_movement[coordinates[0]], self.rotation_angle, self.rotation_power)
        navigation.h_nav(self.vertical_movement[coordinates[1]], self.depth_change, self.h_power)
        
        if self.frame == width_height:
            self.touching_die_counter += 1
            print 'sub is touching, or close to touching die'

    # back_up_from_die ##################################################################################
    def back_up_from_die(self, navigation, coordinates, power, rotation):
        # TODO perhaps can add a way point when all dice are in view to navigate back to
        # when first die is touched
        navigation.m_nav('power', self.move_backward, power)

    # find_die ##################################################################################
    def rotate_to_find_die(self, navigation, power, rotation):
        # TODO create another way to find die
        # just using rotate from gate maneuver to try and find die
        navigation.r_nav(self.rotation_direction, self.rotation_angle, self.rotation_power)
        
    # level_to_die ##################################################################################
    def level_to_die(self):
        pass

    # rotate_to_center ##################################################################################
    def rotate_to_center(self, navigation, coordinates, power, rotation):
        navigation.r_nav(self.rotation_movement[coordinates[0]], self.rotation_angle, self.rotation_power)

    # completed_dice ##################################################################################
    def completed_dice(self):
        check_1st = self.is_1st_die_touched
        check_2nd = self.is_2nd_die_touched

        if check_1st and check_2nd:
            self.is_task_complete = True

        return self.is_task_complete