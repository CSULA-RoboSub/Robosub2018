import rospy
from auv2018.msg import HControl
from auv2018.msg import RControl
from auv2018.msg import MControl


class StatusLogger():
    """
    Prints the status of the Arduino
    is_logging -- (True, False) turns logging on or off
    """

    def __init__(self, is_logging=True):
        self.is_logging = is_logging

        rospy.Subscriber('height_control_status', HControl, self.printHControl)
        rospy.Subscriber('rotation_control_status', RControl, self.printRControl)
        rospy.Subscriber('movement_control_status', MControl, self.printMControl)

    def toggle_logging(self, arg=None):
        """Toggles logging (True, False, empty == toggle)"""

        # Toggles logging if there is no argument passed
        if arg is None:
            if not self.is_logging:
                self.is_logging = True
            else:
                self.is_logging = False
        elif arg or arg == 1:
            self.is_logging = True
        else:
            self.is_logging = False

        print('\nstatus logging set to %d' % self.is_logging)

    def printHControl(self, data):
        """Callback. Prints all the data for HControl"""

        if self.is_logging:
            hStates = {
                0: 'down',
                1: 'staying',
                2: 'up'
            }

            print(
                'state: %s depth: %.2f power: %d'
                % (hStates[data.state], data.depth, data.power)
            )

    def printRControl(self, data):
        """Callback. Prints all the data for RControl"""

        if self.is_logging:
            rStates = {
                0: 'left',  # rotate left
                1: 'staying',
                2: 'right',  # rotate right
                3: 'rotate_front_cam_dist',  # rotate with fcd
                4: 'keep_rotate_front_cam_dist'  # keeping rotating with fcd
            }

            print(
                'state: %s rotation: %.2f power: %d'
                % (rStates[data.state], data.rotation, data.power)
            )

    def printMControl(self, data):
        """Callback. Prints all the data for MControl"""

        if self.is_logging:
            mStates = {
                0: 'off',
                1: 'power',  # adjust with power
                2: 'distance',  # ajust with distance
                3: 'front_cam_center',  # centered with front camera
                4: 'bot_cam_center',  # centered with bottom camera
                5: 'motor_time'  # turn on motor with specific time
            }

            directions = {
                0: 'none',
                1: 'forward',
                2: 'right',
                3: 'backward',
                4: 'left'
            }

            print(
                'state: %s direction: %s power: %d distance: %.2f runningTime: %.2f'
                % (mStates[data.state], directions[data.mDirection], data.power, data.distance, data.runningTime)
            )
