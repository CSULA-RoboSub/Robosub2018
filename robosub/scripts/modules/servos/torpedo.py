import rospy
from robosub.msg import Torpedo
from modules.main.status import log


""" Controls torpedoes"""

sides = {
    'left': 0,
    'right': 1,
    'both': 2
}

pub = rospy.Publisher('torpedo', Torpedo, queue_size=100)

def prime_torpedo(side):
    """
    Prime the torpedo with given side/s
    side -- (str) left, right, both
    """

    if sides['both'] == sides[side]:
        print('prime both torpedoes')
    elif side in sides:
        print('prime %s torpedo' % side)

def fire_torpedo(side):
    """
    Fire the torpedo with given side/s
    side -- (str) left, right, both
    """

    if sides['both'] == sides[side]:
        print('fire both torpedoes')
    elif side in sides:
        print('fire %s torpedo' % side)