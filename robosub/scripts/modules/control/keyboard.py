import rospy
from misc.getch import _Getch
from std_msgs.msg import String
from modules.control.navigation import Navigation
from modules.control.waypoint import Waypoint
from threading import Thread
import time


class Keyboard():

    """Navigate the robosub using keyboard controls
    w: forwards
    a: counter-clockwise
    s: backwards
    d: clockwise
    q: left
    e: right
    r: up
    f: down
    [0-9]: power [1]: 10% [0]: 100%
    `: stop
    c: custom power
    v: custom rotation
    h: set height
    x: exit
    """

    def __init__(self, navigation=None, waypoint=None):
        self.is_killswitch_on = False
        self.multiplier = 40
        self.r_multiplier = 18.0

        # self.navigation = Navigation(self.waypoint)
        if navigation:
            self.navigation = navigation
        else:
            self.navigation = Navigation()
        self.h_power = 100
        self.m_power = 100
        self.r_power = 100
        self.thread_w = None
        self.exit = False
        self.w_time_delay = 5

    def getch(self):
        """Gets keyboard input if killswitch is plugged in"""

        getch = _Getch()
        accepted = ['w', 'a', 's', 'd', 'q', 'e', 'r', 'f', '`']
        response = ''
        char = 0
        last_char = 0
        rotation = self.r_multiplier
        height = 0.0
        if self.is_killswitch_on:
            print(
                '\
                \nw: forwards\
                \na: counter-clockwise\
                \ns: backwards\
                \nd: clockwise\
                \nq: left\
                \ne: right\
                \nr: up\
                \nf: down\
                \n[0-9]: power [1]: 10% [0]: 100%\
                \n`: stop\
                \nm: custom movement power\
                \n,: custom height power\
                \n.: custom rotation power\
                \nv: custom rotation\
                \nh: set height\
                \ng: record waypoint\
                \nt: go to last waypoint\
                \no: set all waypoint start delay\
                \np: run through all waypoints\
                \nk: display current waypoints in queue\
                \nl: clear all waypoints\
                \nx: exit')

            while char != 'x':
                last_char = char
                char = getch()
                if char in accepted:
                    self.navigate(char, rotation, height, last_char)
                elif char.isdigit():
                    if char == '0':
                        self.m_power = int(10) * self.multiplier
                        self.r_power = self.m_power
                        rotation = int(10) * self.r_multiplier
                    else:
                        self.m_power = int(char) * self.multiplier
                        self.r_power = self.m_power
                        rotation = int(char) * self.r_multiplier

                    print('power: %d rotation: %.2f degrees' %
                          (self.m_power, rotation))
                elif char == '.':
                    while not response.isdigit() or int(response) < 0 or int(response) > 400:
                        response = raw_input(
                            '\nEnter a custom rotation power value [0-400]: ')

                    self.r_power = int(response)
                    response = ''
                    print('rotation power: %d' % self.r_power)
                elif char == ',':
                    while not response.isdigit() or int(response) < 0 or int(response) > 400:
                        response = raw_input(
                            '\nEnter a custom height power value [0-400]: ')

                    self.h_power = int(response)
                    response = ''
                    print('height power: %d' % self.h_power)
                elif char == 'm':
                    while not response.isdigit() or int(response) < 0 or int(response) > 400:
                        response = raw_input(
                            '\nEnter a custom movement power value [0-400]: ')

                    self.m_power = int(response)
                    response = ''
                    print('movement power: %d' % self.m_power)
                elif char == 'o':
                    while not response.isdigit() or int(response) < 0 or int(response) > 400:
                        response = raw_input(
                            '\nEnter a time delay in seconds: ')

                    self.w_time_delay = int(response)
                    response = ''
                    print('delay time: %d' % self.w_time_delay)
                elif char == 'v':
                    while True:
                        try:
                            response = raw_input(
                                '\nEnter a custom rotation value [0-180]: ')
                            rotation = float(response)
                            if rotation < 0.0 or rotation > 180.0:
                                raise ValueError
                        except ValueError:
                            pass
                        else:
                            break

                    response = ''
                    print('rotation: %.2f' % rotation)
                elif char == 'h':
                    while True:
                        try:
                            response = raw_input('\nEnter a height: ')
                            height = float(response)
                        except ValueError:
                            pass
                        else:
                            break

                    response = ''
                    print('height: %.2f' % height)
                elif char == 'g':
                    print('Waypoint set at - ')
                    self.navigation.enqueue_current_waypoint()

                elif char == 'l':
                    while True:
                        try:
                            if(self.navigation.is_empty()):
                                print('No waypoints in queue')
                            else:
                                response = raw_input(
                                    '\nAre you sure you want to clear the queue? \n1)YES \n2)NO \n')
                                choice = int(response)
                                if choice == 1:
                                    print('Cleared Waypoints')
                                    self.navigation.clear_waypoints()
                                elif choice == 2:
                                    print('Canceled')
                                else:
                                    raise ValueError
                        except ValueError:
                                pass
                        else:
                                break
                        response = ''
                elif char == 'k':
                    print('These are the current waypoints')
                    self.navigation.display_waypoints()

                elif char == 't':
                    # self.navigation.run_top_stack_waypoint(self.r_power, self.h_power, self.m_power)
                    print('Attempting to go to the most recent waypoint')
                    self.navigation.run_last_queue_waypoint(
                        self.r_power, self.h_power, self.m_power)
                    #print(self.waypoint.is_empty())
                    #self.waypoint.is_empty()
                elif char == 'p':
                    print('Attempting to run through all waypoints')
                    print('waiting %d seconds' % self.w_time_delay)

                    time.sleep(self.w_time_delay)
                    self.navigation.run_queue_waypoints()
                    #self.navigation.run_rot_queue_waypoints()
            self.navigation.set_exit_waypoints(True)
        else:
            print('Magnet is not plugged in.')

    def navigate(self, char, rotation, height, last_char):
        """Navigates robosub with given character input and power"""


        if char == '`':
            self.navigation.cancel_all_nav()
            if char != last_char:
                print('Shut movement off')
            else:
                last_char = char

        elif char == 'w':
            self.navigation.cancel_all_nav()
            #self.navigation.cancel_m_nav()
            self.navigation.m_nav('power', 'forward', self.m_power)

            # self.navigation.m_nav('distance', 'forward', self.m_power, 1)
            if char != last_char:
                print('Going forward with power %.2f' % self.m_power)
            else:
                last_char = char

        elif char == 'a':
            #self.navigation.cancel_all_nav()
            self.navigation.cancel_r_nav()
            self.navigation.r_nav('left',rotation,self.r_power)
            #self.navigation.cancel_and_r_nav('left', rotation, self.r_power)

            if char != last_char:
                print('Rotating left - degrees %d' % rotation)
                print('power %.2f' % self.r_power)
            else:
                last_char = char

        elif char == 's':
            self.navigation.cancel_all_nav()
            #elf.navigation.cancel_m_nav()
            self.navigation.m_nav('power', 'backward', self.m_power)
            if char != last_char:
                print('Going backward with power %.2f' % self.m_power)
            else:
                last_char = char
            # self.navigation.m_nav('distance', 'backward', self.m_power, 1)
        elif char == 'd':
            self.navigation.cancel_all_nav()
            #self.navigation.cancel_r_nav()
            self.navigation.r_nav('left',rotation,self.r_power)
            #self.navigation.cancel_and_r_nav('right', rotation, self.r_power)
            if char != last_char:
                print('Rotating right - degrees %d' % rotation)
                print('power %.2f' % self.r_power)
            else:
                last_char = char

        elif char == 'q':
            self.navigation.cancel_all_nav()
            #self.navigation.cancel_m_nav()

            self.navigation.m_nav('power', 'left', self.m_power)
            if char != last_char:
                print('Strafing left with power %.2f' % self.m_power)
            else:
                last_char = char
            # self.navigation.m_nav('distance', 'left', self.m_power, 1)
        elif char == 'e':
            self.navigation.cancel_all_nav()

            #self.navigation.cancel_m_nav()

            self.navigation.m_nav('power', 'right', self.m_power)
            if char != last_char:
                print('Strafing right with power %.2f' % self.m_power)
            else:
                last_char = char
            # self.navigation.m_nav('distance', 'right', self.m_power, 1)
        elif char == 'r':
            self.navigation.cancel_all_nav()

            #self.navigation.cancel_h_nav(self.h_power)
            self.navigation.h_nav('up', height, self.h_power)
            if char != last_char:
                print('Emerging - %d' % height)
                print('power %.2f' % self.h_power)
            else:
                last_char = char

        elif char == 'f':
            self.navigation.cancel_all_nav()

            #self.navigation.cancel_h_nav(self.h_power)
            self.navigation.h_nav('down', height, self.h_power)
            if char != last_char:
                print('Submerging - %d' % height)
                print('power %.2f' % self.h_power)
            else:
                last_char = char

    def set_power(self, **power_type):
        """
        Sets the power to a value set_power(power_type=value)
        power_type: h_power, m_power, r_power
        """

        if 'h_power' in power_type:
            self.h_power = power_type['h_power']
        if 'm_power' in power_type:
            self.h_power = power_type['m_power']
        if 'r_power' in power_type:
            self.h_power = power_type['r_power']

    def start(self):
        """Allows keyboard navigation when killswitch is plugged in"""

        self.is_killswitch_on = True
        # self.navigation.start()

    def stop(self):
        """Stops keyboard navigation when killswitch is unplugged"""

        self.is_killswitch_on = False
        self.navigation.stop()
