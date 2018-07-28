import rospy

from robosub.msg import MControl

class DiceManeuver():
    def __init__(self):
        ################ THRESHOLD VARIABLES ################
        self.touching_die_threshold = 100
        self.nothing_found_threshold = 100
        self.back_up_threshold = 100

        ################ FLAG VARIABLES ################
        self.is_moving_forward = False
        self.is_rotated_to_center = False
        self.is_1st_die_touched = False
        self.is_2nd_die_touched = False
        self.is_task_complete = False

        ################ TIMER/COUNTER VARIABLES ################
        self.touching_die_counter = 0
        self.nothing_found_counter = 0
        self.back_up_counter = 0
        self.dice_touched = 0

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

        self.m_power_forward_horizontal = {
            -1: 85,
             0: 140,
             1: 85
        }

        ################ AUV MOBILITY VARIABLES ################
        self.rotation_angle = 5
        self.move_forward = 'forward'
        self.move_backward = 'backward'
        self.rotation_power = 70
        self.depth_change = .5
        self.r_power=75
        self.h_power=100
        self.m_power=120
        # TODO remove soon
        self.rotation_direction = 'right'
        self.m_state_is_moving_forward = 0
        self.m_distance = 0.5

        ################ FRAME VARIABLES ################
        self.frame = (744, 480)
        self.close_enough_values = (100, 100)
        # self.frame = (640, 480)

        ################ ROS STUFF ######################
        rospy.Subscriber('movement_control_status', MControl, self.m_status_callback, queue_size=100)

    # reset ##################################################################################
    def reset(self):
        self.is_moving_forward = False
        self.is_rotated_to_center = False
        self.is_1st_die_touched = False
        self.is_2nd_die_touched = False
        self.is_task_complete = False

        self.touching_die_counter = 0
        self.nothing_found_counter = 0
        self.back_up_counter = 0
        self.m_state_is_moving_forward = 0
        self.dice_touched = 0
    
    # reset_after_1st_die ##################################################################################
    def reset_after_1st_die(self):
        self.is_moving_forward = False
        self.is_rotated_to_center = False

        self.touching_die_counter = 0
        self.nothing_found_counter = 0
        self.back_up_counter = 0
        self.m_state_is_moving_forward = 0

    # touch_die ##################################################################################
    def touch_die(self, navigation, coordinates, power, rotation, width_height):
        if width_height[1] < self.close_enough_values[1] and not self.is_moving_forward:
            navigation.cancel_and_m_nav('power', self.horizontal_move_with_forward[coordinates[0]], self.m_power_forward_horizontal[coordinates[0]])
            # navigation.r_nav(self.rotation_movement[coordinates[0]], self.rotation_angle, self.rotation_power)
            navigation.cancel_and_h_nav(self.vertical_movement[coordinates[1]], self.depth_change, self.h_power)
        
        elif width_height[1] >= self.close_enough_values[1] and not self.is_moving_forward:
            self.is_moving_forward = True
            self.cance_and_m_nav('distance', 'forward', self.m_power, self.m_distance)
            self.m_state_is_moving_forward = 1

        # if self.frame == width_height:
        #     self.touching_die_counter += 1
        #     print 'touching die counter {}'.format(self.touching_die_counter)
        #     print 'sub is touching, or close to touching die'

    # back_up_from_die ##################################################################################
    # def back_up_from_die(self, navigation, power):
    #     # TODO perhaps can add a way point when all dice are in view to navigate back to
    #     # when first die is touched
    #     navigation.cancel_and_m_nav('power', self.move_backward, power)
    #     self.back_up_counter += 1

    # find_die ##################################################################################
    def rotate_to_find_die(self, navigation, power, rotation):
        # TODO create another way to find die
        # just using rotate from gate maneuver to try and find die
        navigation.cancel_and_r_nav(self.rotation_direction, self.rotation_angle, self.rotation_power)
        
    # level_to_die ##################################################################################
    def level_to_die(self):
        pass

    # rotate_to_center ##################################################################################
    def rotate_to_center(self, navigation, coordinates, power, rotation):
        if not coordinates[0] == 0:
            navigation.cancel_and_r_nav(self.rotation_movement[coordinates[0]], self.rotation_angle, self.rotation_power)
        else:
            navigation.cancel_r_nav()
        self.nothing_found_counter = 0

    # completed_dice ##################################################################################
    def completed_dice_check(self):
        check_1st = self.is_1st_die_touched
        check_2nd = self.is_2nd_die_touched

        if check_1st and check_2nd:
            self.is_task_complete = True

        return self.is_task_complete

    # rotate ##################################################################################
    def rotate(self, navigation, r_power, rotation):
        navigation.cancel_and_r_nav(self.rotation_direction, self.rotation_angle, self.r_power)
        
    # no_shape_found ##################################################################################
    def no_shape_found(self, navigation, coordinates, power, rotation, width_height):
        if self.nothing_found_counter >= self.nothing_found_threshold:
            self.rotate(navigation, power, rotation)
        self.nothing_found_counter += 1

    # centered_and_shape_found ##################################################################################
    def centered_and_shape_found(self, navigation, coordinates, power, rotation, width_height):
        self.nothing_found_counter = 0

        if not self.is_moving_forward:
            self.touch_die(navigation, coordinates, power, rotation, width_height)

        # if self.back_up_counter > self.back_up_threshold:
        #     if not self.is_1st_die_touched:
        #         self.is_1st_die_touched = True
        #     else:
        #         self.is_2nd_die_touched = True

    #movement control status callback
    def m_status_callback(self, movement_status):
        # print(rotation_status)
        if self.is_moving_forward:

            if movement_status.state == 0 and self.m_state_is_moving_forward == 1:
                # print('in state 1')
                self.m_nav('distance', 'backward', self.m_power, self.m_distance)
                self.m_state_is_moving_forward = 2
                # print("backward wp state 2")

            if movement_status.state == 0 and self.m_state_is_moving_forward == 2:
                self.dice_touched += 1
                self.reset_after_1st_die()
