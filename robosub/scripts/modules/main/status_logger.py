import rospy
from datetime import datetime
from robosub.msg import HControl
from robosub.msg import RControl
from robosub.msg import MControl


class StatusLogger():
    """
    Prints the status of the Arduino
    is_logging -- (True, False) turns logging on or off
    """

    def __init__(self, is_logging=True):
        self.is_logging = is_logging

        self.file = open('logs/' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_log.txt', 'a')

        rospy.Subscriber('height_control', HControl, self.printHControl)
        rospy.Subscriber('rotation_control', RControl, self.printRControl)
        rospy.Subscriber('movement_control', MControl, self.printMControl)

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
                2: 'up',
                4: 'unlock',
                5: 'lock'
            }

            log = (
                'state: %s depth: %.2f power: %d'
                % (hStates[data.state], data.depth, data.power)
            )

            # print(log)
            self.file.write(log + '\n')

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

            log = (
                'state: %s rotation: %.2f power: %d'
                % (rStates[data.state], data.rotation, data.power)
            )

            # print(log)
            self.file.write(log + '\n')

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

            log = (
                'state: %s direction: %s power: %d distance: %.2f runningTime: %.2f'
                % (mStates[data.state], directions[data.mDirection], data.power, data.distance, data.runningTime)
            )

            # print(log)
            self.file.write(log + '\n')

    def start(self):
<<<<<<< HEAD
        self.file = open('logs/' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_log.txt', 'a')
=======
        # self.file = open('logs/' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_log.txt', 'a')
        pass
>>>>>>> dev-henry

    def stop(self):
        self.file.close()
