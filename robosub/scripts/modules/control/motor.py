import rospy
from robosub.msg import HControl
from modules.main.status import log


class Motor():
    """Controls motors"""

    def __init__(self, state=5):
        self.is_killswitch_on = False

        self.pub = rospy.Publisher('height_control', HControl, queue_size=100)

        self.state = state

        self.h_control = HControl()
        self.h_control.state = self.state
        self.h_control.depth = 0
        self.h_control.power = 0

    def get_state(self):
        return self.state

    def toggle_state(self, arg=None):
        """Toggles the state of the motors (4 == on, 5 == off, empty == toggle)"""

        if arg != 4 and arg != 5:
            return

        # Toggles the state if there is no argument passed
        if arg is None:
            if self.state == 5:
                self.state = 4
            else:
                self.state = 5
        else:
            self.state = arg

        print('\nmotor state set to %d' % self.state)
        log('\nmotor state set to %d' % self.state)

        if self.is_killswitch_on:
            self.pub_motor_state(self.state)

    def pub_motor_state(self, state):
        """ Private method used to publish given motor state"""
        self.h_control.state = state

        self.pub.publish(self.h_control)

        print('\nmotor state published %d' % state)

    def start(self):
        """Starts motors with set preferences when killswitch is plugged in"""

        self.is_killswitch_on = True

        self.pub_motor_state(self.state)

    def stop(self):
        """Stops motors when killswitch is unplugged"""

        self.is_killswitch_on = False

        self.pub_motor_state(5)
