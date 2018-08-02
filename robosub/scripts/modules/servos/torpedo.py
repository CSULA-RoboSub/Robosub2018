import rospy
from robosub.msg import Torpedo
from modules.main.status import log


""" Controls torpedoes"""

sides = {
    'left': 0,
    'right': 1,
    'both': 2
}


def prime_torpedo(side):
    """
    Prime the torpedo with given side/s
    side -- (str) left, right, both
    """

    # left side
    if side == 0 or sides['left'] == sides[side]:
        print('prime left')
    # right side
    elif side == 1 or sides['right'] == sides[side]:
        print('prime right')
    # both sides
    elif side == 2 or sides['both'] == sides[side]:
        print('prime both')


def shoot_torpedo(side):
    """
    Shoot the torpedo with given side/s
    side -- (str) left, right, both
    """

    # left side
    if side == 0 or sides['left'] == sides[side]:
        print('shoot left')
    # right side
    elif side == 1 or sides['right'] == sides[side]:
        print('shoot right')
    # both sides
    elif side == 2 or sides['both'] == sides[side]:
        print('shoot both')