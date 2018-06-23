import rospy
# from auv2018.msg import Navigate
from robosub.msg import HControl
from robosub.msg import RControl
from robosub.msg import MControl


class Navigation():
    """
    AUV 2017 Version
    Controls thrusters to move or point AUV to a certain direction given power and direction or rotational values
    """

    def __init__(self):
        self.is_killswitch_on = False

        self.pub_h_nav = rospy.Publisher('height_control', HControl, queue_size=100)
        self.pub_r_nav = rospy.Publisher('rotation_control', RControl, queue_size=100)
        self.pub_m_nav = rospy.Publisher('movement_control', MControl, queue_size=100)

        # rospy.init_node('navigation_node', anonymous=True)
        rospy.Subscriber('current_rotation_status', RControl, self.rCallBack, queue_size=100)

        self.h_control = HControl()
        self.r_control = RControl()
        self.m_control = MControl()

        # used for HControl (int state, float depth, int power) #######################################
        self.hStates = {
            'down': 0,
            'staying': 1,
            'up': 2,
            'unlock': 4,
            'lock': 5
        }
        self.hState = None  # state

        self.depth = None  # depth (nonstop moving: -1, moving distance: x)

        self.hPower = None  # power

        # used for RControl (int state, float rotation, int power) ####################################
        self.rStates = {
            'left': 0,  # rotate left
            'staying': 1,
            'right': 2,  # rotate right
            'rotate_front_cam_dist': 3,  # rotate with fcd
            'keep_rotate_front_cam_dist': 4  # keeping rotating with fcd
        }
        self.rState = None  # state

        self.rotation = None  # rotation (nonstop rotating: -1, rotate degree: x)

        self.rPower = None  # power

        # used for MControl (int state, int mDirection, float power, float distance) #######
        self.mStates = {
            'off': 0,
            'power': 1,  # adjust with power
            'distance': 2,  # ajust with distance
            'front_cam_center': 3,  # centered with front camera
            'bot_cam_center': 4,  # centered with bottom camera
            'motor_time': 5  # turn on motor with specific time
        }
        self.mState = None  # state

        self.directions = {
            'none': 0,
            'forward': 1,
            'right': 2,
            'backward': 3,
            'left': 4
        }

        self.mDirection = None  # mDirection

        self.mPower = None  # power (none: 0, motor power: x)

        self.distance = None  # distance (distance away from the object: x)

        self.runningTime = None  # runningTime (time for the motor to turn on)

        self.is_running_waypoint = False
        self.w_distance_m = 0
        self.w_power_m = 100
    def set_h_nav(self, hState, depth, hPower):
        """
        hState -- 'down': 0, 'staying': 1, 'up': 2
        depth -- nonstop moving: -1, moving distance: x
        hPower -- int
        """

        if hState.isdigit():
            self.hState = hState
        else:
            self.hState = self.hStates[hState]

        self.depth = depth

        self.hPower = hPower

    def set_r_nav(self, rState, rotation, rPower):
        """
        rState -- 'left': 0, 'staying': 1, 'right': 2, 'rotate_front_cam_dist': 3, 'keep_rotate_front_cam_dist': 4
        rotation -- nonstop rotating: -1, rotate degree: x
        rPower -- int
        """

        if rState.isdigit():
            self.rState = rState
        else:
            self.rState = self.rStates[rState]

        self.rotation = rotation

        self.rPower = rPower

    def set_m_nav(self, mState, mDirection, power, value=0.0):
        """
        mState -- 'off': 0, 'power': 1, 'distance': 2, 'front_cam_center': 3, 'bot_cam_center': 4, 'motor_time': 5
        mDirection -- 'none': 0, 'forward': 1, 'right': 2, 'backward': 3, 'left': 4
        power -- none: 0, motor power: x
        value -- based on mState
            (2)distance: distance away from the object: x
            (5)runningTime: time for the motor to turn on
        """

        if mState.isdigit():
            self.mState = mState
        else:
            self.mState = self.mStates[mState]

        if mDirection.isdigit():
            self.mDirection = mDirection
        else:
            self.mDirection = self.directions[mDirection]

        self.mPower = power
        self.distance = 0.0
        self.runningTime = 0.0

        if self.mState == self.mStates['distance']:
            self.distance = value
        elif self.mState == self.mStates['motor_time']:
            self.runningTime = value

    def cancel_m_nav(self, power = 160):
        self.m_nav('off', 'none', power)

    def cancel_h_nav(self, power = 100):
        self.h_nav('staying', 0, power)

    def cancel_r_nav(self, power = 160):
        self.r_nav('staying', 0, power)

    def h_nav(self, hState=None, depth=None, hPower=None):
        """
        Start horizontal navigation given hState and depth when killswitch is on.
        hState -- 'down': 0, 'staying': 1, 'up': 2
        depth -- nonstop moving: -1, moving distance: x
        hPower -- int
        """

        if self.is_killswitch_on:

            if hState is not None or depth is not None or hPower is not None:
                self.set_h_nav(hState, depth, hPower)

            self.h_control.state = self.hState
            self.h_control.depth = self.depth
            self.h_control.power = self.hPower

            self.pub_h_nav.publish(self.h_control)
            # self.ros_sleep()
            # rospy.sleep(.1)

            # print('state: %d depth: %.2f power: %d' % (self.hState, self.depth, self.hPower))

    def r_nav(self, rState=None, rotation=None, rPower=None):
        """
        Start rotational navigation given rState and rotation when killswitch is on.
        rState -- 'left': 0, 'staying': 1, 'right': 2, 'rotate_front_cam_dist': 3, 'keep_rotate_front_cam_dist': 4
        rotation -- nonstop rotating: -1, rotate degree: x
        rPower -- int
        """

        if self.is_killswitch_on:

            if rState is not None or rotation is not None or rPower is not None:
                self.set_r_nav(rState, rotation, rPower)

            self.r_control.state = self.rState
            self.r_control.rotation = self.rotation
            self.r_control.power = self.rPower

            self.pub_r_nav.publish(self.r_control)
            # self.ros_sleep()
            # rospy.sleep(.1)

            # print('state: %d rotation: %.2f power: %d' % (self.rState, self.rotation, self.rPower))

    def m_nav(self, mState=None, mDirection=None, power=None, value=None):
        """
        Start movement navigation given mState, mDirection, and power/distance/runningTime when killswitch is on.
        mState -- 'off': 0, 'power': 1, 'distance': 2, 'front_cam_center': 3, 'bot_cam_center': 4, 'motor_time': 5
        mDirection -- 'none': 0, 'forward': 1, 'right': 2, 'backward': 3, 'left': 4
        power -- none: 0, motor power: x
        value -- based on mState
            (2)distance: distance away from the object: x
            (5)runningTime: time for the motor to turn on
        """

        if self.is_killswitch_on:

            if mState is not None or mDirection is not None or power is not None:
                self.set_m_nav(mState, mDirection, power, value)

            self.m_control.state = self.mState
            self.m_control.mDirection = self.mDirection
            self.m_control.power = self.mPower
            self.m_control.distance = self.distance
            self.m_control.runningTime = self.runningTime

            self.pub_m_nav.publish(self.m_control)
            # self.ros_sleep()
            # rospy.sleep(.1)

            # print(
            #     'state: %d direction: %d power: %.2f distance: %.2f runningTime: %.2f'
            #     % (self.mState, self.mDirection, self.mPower, self.distance, self.runningTime)
            # )

    def start(self):
        """Starts navigation with set preferences when killswitch is plugged in"""

        self.is_killswitch_on = True

    def stop(self):
        """Stops navigation when killswitch is unplugged"""

        self.is_killswitch_on = False

    def ros_sleep(self, time = 0.05):
        if time:
            rospy.sleep(time)
        else:
            rospy.sleep()

    def ros_rate(self, hz = 100):
        rospy.Rate(hz)


    def rCallBack(self, rotation_status):
        if self.is_running_waypoint:
            if rotation_status.state == 1:
                self.is_running_waypoint = False
                self.m_nav('distance', 'forward', self.w_power_m, self.w_distance_m)

    def go_waypoint(self, direction_r, degree_r, distance_r, power_r, direction_h, distance_h, power_h, distance_m, power_m):
        if not direction or not degree or not distance or not depth or not power or not h_power:
            return

        print('going to waypoint')
        self.is_running_waypoint = True
        self.r_nav(direction_r, degree_r, power_r)
        self.h_nav(direction_h, distance_h, power_h)
        self.w_distance_m = distance_m
        self.w_power_m = power_m