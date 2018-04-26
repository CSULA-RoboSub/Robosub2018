import rospy
from std_msgs.msg import Int8


class Motor():
    """Controls motors"""

    def __init__(self, state=0):
        self.is_killswitch_on = False

        self.pub = rospy.Publisher('motor_state', Int8, queue_size=10)

        self.state = state

    def get_state(self):
        return self.state

    def toggle_state(self, arg=None):
        """Toggles the state of the motors (1 == on, 0 == off, empty == toggle)"""

        # Toggles the state if there is no argument passed
        if arg is None:
            if self.state == 0:
                self.state = 1
            else:
                self.state = 0
        else:
            self.state = arg

        print('\nmotor state set to %d' % self.state)

        if self.is_killswitch_on:
            self.pub_motor_state(self.state)

    def pub_motor_state(self, state):
        """ Private method used to publish given motor state"""

        self.pub.publish(state)
        rospy.sleep(.1)

        print('\nmotor state published %d' % state)

    def start(self):
        """Starts motors with set preferences when killswitch is plugged in"""

        self.is_killswitch_on = True

        self.pub_motor_state(self.state)

    def stop(self):
        """Stops motors when killswitch is unplugged"""

        self.is_killswitch_on = False

        self.pub_motor_state(0)
