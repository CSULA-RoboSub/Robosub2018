import rospy
from datetime import datetime
from robosub.msg import HControl
from robosub.msg import RControl
from robosub.msg import MControl

""" Prints and logs status to file"""

is_logging = False

file = open('logs/' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_log.txt', 'a')


def log(data):
    """
    Prints to console and logs to file
    data -- str
    """

    print(data)
    try:
        file.write(data + '\n')
    except ValueError:
        pass


def printHControl(data):
    """Callback. Prints all the data for HControl"""

    if is_logging:
        hStates = {
            0: 'down',
            1: 'staying',
            2: 'up',
            4: 'unlock',
            5: 'lock'
        }

        text = (
            '%s depth: %.2f /power: %d'
            % (hStates[data.state], data.depth, data.power)
        )

        # text = (
        #     'state: %s depth: %.2f power: %d'
        #     % (hStates[data.state], data.depth, data.power)
        # )

        # print(text)
        try:
            file.write(text + '\n')
        except ValueError:
            pass


def printRControl(data):
    """Callback. Prints all the data for RControl"""

    if is_logging:
        rStates = {
            0: 'left',  # rotate left
            1: 'staying',
            2: 'right',  # rotate right
            3: 'rotate_front_cam_dist',  # rotate with fcd
            4: 'keep_rotate_front_cam_dist'  # keeping rotating with fcd
        }

        text = (
            'rotate %s %.2f degrees /power: %d'
            % (rStates[data.state], data.rotation, data.power)
        )
        # text = (
        #     'state: %s rotation: %.2f power: %d'
        #     % (rStates[data.state], data.rotation, data.power)
        # )

        # print(text)
        try:
            file.write(text + '\n')
        except ValueError:
            pass


def printMControl(data):
    """Callback. Prints all the data for MControl"""

    if is_logging:
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

        # TODO make conditional statements for stopping and different states
        text = (
            '%s /power: %d /distance: %.2f /runningTime: %.2f /state: %s'
            % (directions[data.mDirection], data.power, data.distance, data.runningTime, mStates[data.state])
        )
        # text = (
        #     'state: %s direction: %s power: %d distance: %.2f runningTime: %.2f'
        #     % (mStates[data.state], directions[data.mDirection], data.power, data.distance, data.runningTime)
        # )

        # print(text)
        try:
            file.write(text + '\n')
        except ValueError:
            pass


rospy.Subscriber('height_control', HControl, printHControl)
rospy.Subscriber('rotation_control', RControl, printRControl)
rospy.Subscriber('movement_control', MControl, printMControl)


def start():
    # self.file = open('logs/' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_log.txt', 'a')
    pass


def stop():
    file.close()