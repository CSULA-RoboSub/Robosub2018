import rospy
from robosub.msg import Dropper
from modules.main.status import log

# def close_both_gates(self):
#     print 'close both gates'

# def open_both_gates(self):
#     print 'open both gates'

# def drop_one_ball(self):
#     print 'drop one ball'

""" Controls dropper"""

CLOSE_BOTH_GATES = 0
OPEN_BOTH_GATES = 1
DROP_ONE_BALL = 2
DROP_OPTIONS = {
    0: 'CLOSE BOTH GATES',
    1: 'OPEN BOTH GATES',
    2: 'DROP ONE BALL'
}

dropper = Dropper()
pub = rospy.Publisher('dropper', Dropper, queue_size=100)

def drop_control(state):
    """
    Changes dropper state
    """
    state = int(state)
    if state in DROP_OPTIONS:
        dropper.state = state
        pub.publish(dropper)
        print DROP_OPTIONS[state]