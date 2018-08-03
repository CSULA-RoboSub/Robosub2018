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

def drop_control(state):
    """
    Changes dropper state
    """
    state = int(state)
    
    if state == 0:
        print 'close both gates'
    elif state == 1:
        print 'open both gates'
    elif state == 2:
        print 'drop one ball'