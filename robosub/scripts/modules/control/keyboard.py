from misc.getch import _Getch
from modules.control.navigation import Navigation


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
        self.navigation = Navigation()

    def getch(self):
        """Gets keyboard input if killswitch is plugged in"""

        getch = _Getch()
        accepted = ['w', 'a', 's', 'd', 'q', 'e', 'r', 'f', '`']
        response = ''
        char = 0
        power = self.multiplier
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
                \nc: custom power\
                \nv: custom rotation\
                \nh: set height\
                \nx: exit')

            while char != 'x':
                char = getch()

                if char in accepted:
                    self.navigate(char, power, rotation, height)
                elif char.isdigit():
                    if char == '0':
                        power = int(10) * self.multiplier
                        rotation = int(10) * self.r_multiplier
                    else:
                        power = int(char) * self.multiplier
                        rotation = int(char) * self.r_multiplier

                    print('power: %d rotation: %.2f degrees' % (power, rotation))
                elif char == 'c':
                    while not response.isdigit() or int(response) < 0 or int(response) > 400:
                        response = raw_input('\nEnter a custom power value [0-400]: ')

                    power = int(response)
                    response = ''
                    print('power: %d' % power)
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

        else:
            print('Magnet is not plugged in.')

    def navigate(self, char, power, rotation, height):
        """Navigates robosub with given character input and power"""

        if char == '`':
            self.navigation.cancel_h_nav()
            self.navigation.ros_sleep()
            self.navigation.cancel_r_nav()
            self.navigation.ros_sleep()
            self.navigation.cancel_m_nav()
        elif char == 'w':
            self.navigation.cancel_m_nav()
            self.navigation.ros_sleep()
            self.navigation.m_nav('power', 'forward', power)
        elif char == 'a':
            self.navigation.cancel_r_nav()
            self.navigation.ros_sleep()
            self.navigation.r_nav('left', rotation, power)
        elif char == 's':
            self.navigation.cancel_m_nav()
            self.navigation.ros_sleep()
            self.navigation.m_nav('power', 'backward', power)
        elif char == 'd':
            self.navigation.cancel_r_nav()
            self.navigation.ros_sleep()
            self.navigation.r_nav('right', rotation, power)
        elif char == 'q':
            self.navigation.cancel_m_nav()
            self.navigation.ros_sleep()
            self.navigation.m_nav('power', 'left', power)
        elif char == 'e':
            self.navigation.cancel_m_nav()
            self.navigation.ros_sleep()
            self.navigation.m_nav('power', 'right', power)
        elif char == 'r':
            self.navigation.cancel_h_nav()
            self.navigation.ros_sleep()
            self.navigation.h_nav('up', height, 120)
        elif char == 'f':
            self.navigation.cancel_h_nav()
            self.navigation.ros_sleep()
            self.navigation.h_nav('down', height, 120)

        self.navigation.ros_sleep()

    def start(self):
        """Allows keyboard navigation when killswitch is plugged in"""

        self.is_killswitch_on = True
        self.navigation.start()

    def stop(self):
        """Stops keyboard navigation when killswitch is unplugged"""

        self.is_killswitch_on = False
        self.navigation.stop()
