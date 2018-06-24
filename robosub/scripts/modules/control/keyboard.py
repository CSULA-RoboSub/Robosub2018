from misc.getch import _Getch
from modules.control.navigation import Navigation
from modules.control.waypoint import Waypoint
from threading import Thread

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

    def __init__(self):
        self.is_killswitch_on = False
        self.multiplier = 40
        self.r_multiplier = 18.0
        # self.waypoint = Waypoint()
        # self.navigation = Navigation(self.waypoint)
        self.navigation = Navigation()
        self.h_power = 100
        self.m_power = 100
        self.r_power = 100
        self.thread_w = None
        self.exit = False
    def getch(self):
        """Gets keyboard input if killswitch is plugged in"""

        getch = _Getch()
        accepted = ['w', 'a', 's', 'd', 'q', 'e', 'r', 'f', '`']
        response = ''
        char = 0
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
                \np: run through all waypoints\
                \nx: exit')

            while char != 'x':
                char = getch()

                if char in accepted:
                    self.navigate(char, rotation, height)
                elif char.isdigit():
                    if char == '0':
                        self.m_power = int(10) * self.multiplier
                        self.r_power = self.m_power
                        rotation = int(10) * self.r_multiplier
                    else:
                        self.m_power = int(char) * self.multiplier
                        self.r_power = self.m_power
                        rotation = int(char) * self.r_multiplier

                    print('power: %d rotation: %.2f degrees' % (self.m_power, rotation))
                elif char == '.':
                    while not response.isdigit() or int(response) < 0 or int(response) > 400:
                        response = raw_input('\nEnter a custom rotation power value [0-400]: ')

                    self.r_power = int(response)
                    response = ''
                    print('rotation power: %d' % self.r_power)
                elif char == ',':
                    while not response.isdigit() or int(response) < 0 or int(response) > 400:
                        response = raw_input('\nEnter a custom height power value [0-400]: ')

                    self.h_power = int(response)
                    response = ''
                    print('height power: %d' % self.h_power)
                elif char == 'm':
                    while not response.isdigit() or int(response) < 0 or int(response) > 400:
                        response = raw_input('\nEnter a custom movement power value [0-400]: ')

                    self.m_power = int(response)
                    response = ''
                    print('movement power: %d' % self.m_power)
                elif char == 'v':
                    while True:
                        try:
                            response = raw_input('\nEnter a custom rotation value [0-180]: ')
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
                    #record waypoint
                    # cur_x, cur_y, cur_depth = self.waypoint.get_position()
                    # print('pushed (x,y,depth): %.2f, %.2f, %.2f' %(cur_x,cur_y,cur_depth))
                    # self.waypoint.push(cur_x, cur_y, cur_depth)
                    self.navigation.push_current_waypoint()
                elif char == 't':
                    #travel to last waypoint
                    # if not self.waypoint.is_empty():
                    #     last_x, last_y, last_depth = self.waypoint.pop()
                    #     direction_r, degree_r, distance_m = self.waypoint.get_directions(last_x, last_y)
                    #     direction_h, distance_h = self.waypoint.get_depth_directions(last_depth)
                    #     self.navigation.go_waypoint(direction_r, degree_r, self.r_power, direction_h, distance_h, self.h_power, distance_m, self.m_power)
                    self.navigation.run_top_stack_waypoint(self.r_power, self.h_power, self.m_power)
                elif char == 'p':
                    # self.thread_w=Thread(target=self.run_all_waypoints)
                    # self.thread_w.start()
                    self.navigation.run_stack_waypoints_async(self.r_power, self.h_power, self.m_power)
            self.navigation.set_exit_waypoints(True)
        else:
            print('Magnet is not plugged in.')

    # def run_all_waypoints(self):
    #     print('waiting 4 seconds')
    #     self.navigation.ros_sleep(4)
    #     print('running waypoints...')
    #     while not self.waypoint.is_empty() and not self.exit:
    #         if not self.navigation.is_running_waypoint():
    #             # print('running a waypoint')
    #             #travel to waypoint at top of stack if not running one
    #             last_x, last_y, last_depth = self.waypoint.pop()
    #             direction_r, degree_r, distance_m = self.waypoint.get_directions(last_x, last_y)
    #             direction_h, distance_h = self.waypoint.get_depth_directions(last_depth)
    #             self.navigation.go_waypoint(direction_r, degree_r, self.r_power, direction_h, distance_h, self.h_power, distance_m, self.m_power)
    #     print('finished running all waypoints')


    def navigate(self, char, rotation, height):
        """Navigates robosub with given character input and power"""

        if char == '`':
            self.navigation.cancel_h_nav(self.h_power)
            self.navigation.cancel_r_nav(self.r_power)
            self.navigation.cancel_m_nav(self.m_power)
        elif char == 'w':
            self.navigation.cancel_m_nav()
            self.navigation.m_nav('power', 'forward', self.m_power)
            # self.navigation.m_nav('distance', 'forward', self.m_power, 1)
        elif char == 'a':
            self.navigation.cancel_r_nav()
            self.navigation.r_nav('left', rotation, self.r_power)
        elif char == 's':
            self.navigation.cancel_m_nav()
            self.navigation.m_nav('power', 'backward', self.m_power)
            # self.navigation.m_nav('distance', 'backward', self.m_power, 1)
        elif char == 'd':
            self.navigation.cancel_r_nav()
            self.navigation.r_nav('right', rotation, self.r_power)
        elif char == 'q':
            self.navigation.cancel_m_nav()
            self.navigation.m_nav('power', 'left', self.m_power)
            # self.navigation.m_nav('distance', 'left', self.m_power, 1)
        elif char == 'e':
            self.navigation.cancel_m_nav()
            self.navigation.m_nav('power', 'right', self.m_power)
            # self.navigation.m_nav('distance', 'right', self.m_power, 1)
        elif char == 'r':
            self.navigation.cancel_h_nav(self.h_power)
            self.navigation.h_nav('up', height, self.h_power)
        elif char == 'f':
            self.navigation.cancel_h_nav(self.h_power)
            self.navigation.h_nav('down', height, self.h_power)

    def start(self):
        """Allows keyboard navigation when killswitch is plugged in"""

        self.is_killswitch_on = True
        self.navigation.start()

    def stop(self):
        """Stops keyboard navigation when killswitch is unplugged"""

        self.is_killswitch_on = False
        self.navigation.stop()
