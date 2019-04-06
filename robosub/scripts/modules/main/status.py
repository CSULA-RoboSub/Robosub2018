import rospy
from datetime import datetime
import os
from robosub.msg import HControl
from robosub.msg import RControl
from robosub.msg import MControl

""" Prints and logs status to file"""

is_logging = False
file_name = ('logs/' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_log.txt')
file = open(file_name, 'a')


def log(data):
    """
    Prints to console and logs to file
    data -- str
    """

    if is_logging:
        print(data)
        try:
            file.write(data + '\n')
        except ValueError:
            return


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
            return


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
            return


def printMControl(data):
    """Callback. Prints all the data for MControl"""

    if is_logging:
        mStates = {
            0: 'off',
            1: 'power',  # adjust with power
            2: 'distance',  # ajust with distance
            3: 'motor_time'  # turn on motor with specific time
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
            return


# rospy.Subscriber('height_control', HControl, printHControl)
# rospy.Subscriber('rotation_control', RControl, printRControl)
# rospy.Subscriber('movement_control', MControl, printMControl)


def start():
    return


def stop():
    file.close()
    try:
        if os.stat(file_name).st_size == 0:
            os.remove(file_name)
        else:
            print(file_name + ' created.')
    except OSError:
        return
