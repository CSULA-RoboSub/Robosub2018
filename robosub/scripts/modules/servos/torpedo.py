import rospy
from robosub.msg import Torpedo
from modules.main.status import log


""" Controls torpedoes"""

PRIME_TORPEDO = 0
FIRE_TORPEDO = 1
SIDES = {
    'left': 0,
    'right': 1
}

torpedo = Torpedo()
pub = rospy.Publisher('torpedo', Torpedo, queue_size=100)

def prime_torpedo(side):
    """
    Prime the torpedo with given side/s
    side -- (str) left, right
    """

    if side in SIDES:
        torpedo.state = PRIME_TORPEDO
        torpedo.torpedo_number = SIDES[side]
        # pub(torpedo)
        print('prime %s torpedo' % side)

def fire_torpedo(side):
    """
    Fire the torpedo with given side/s
    side -- (str) left, right
    """

    if side in SIDES:
        torpedo.state = FIRE_TORPEDO
        torpedo.torpedo_number = SIDES[side]
        # pub(torpedo)
        print('fire %s torpedo' % side)
