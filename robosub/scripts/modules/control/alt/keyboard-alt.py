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
    [0-9]: power
    c: custom power
    x: exit
    """

    def __init__(self):
        self.is_killswitch_on = False
        self.multiplier = 40
        self.navigation = Navigation()

    def getch(self):
        """Gets keyboard input if killswitch is plugged in"""

        getch = _Getch()
        accepted = ['w', 'a', 's', 'd', 'q', 'e', 'r', 'f']
        response = ''
        char = 0
        power = self.multiplier

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
                \n[0-9]: power\
                \nc: custom power\
                \nx: exit')

            while char != 'x':
                char = getch()

                if char in accepted:
                    self.navigate(char, power)
                elif char.isdigit():
                    if char == '0':
                        power = int(10) * self.multiplier
                    else:
                        power = int(char) * self.multiplier

                    print('power is changed to %s' % power)
                elif char == 'c':
                    while not response.isdigit() or int(response) < 0 or int(response) > 400:
                        response = raw_input('\nEnter a custom power value [0-400]: ')

                    power = response
                    response = ''
                    print('power is changed to %s' % power)

        else:
            print('Magnet is not plugged in.')

    def navigate(self, char, power):
        """Navigates robosub with given character input and power"""

        if char == 'w':
            self.navigation.navigate(0, power, 0, 0.0)
        elif char == 'a':
            self.navigation.navigate(0, 0, 0, -10.0)
        elif char == 's':
            self.navigation.navigate(0, -power, 0, 0.0)
        elif char == 'd':
            self.navigation.navigate(0, 0, 0, 10.0)
        elif char == 'q':
            self.navigation.navigate(-power, 0, 0, 0.0)
        elif char == 'e':
            self.navigation.navigate(power, 0, 0, 0.0)
        elif char == 'r':
            self.navigation.navigate(0, 0, power, 0.0)
        elif char == 'f':
            self.navigation.navigate(0, 0, -power, 0.0)

    def start(self):
        """Allows keyboard navigation when killswitch is plugged in"""

        self.is_killswitch_on = True
        self.navigation.start()

    def stop(self):
        """Stops keyboard navigation when killswitch is unplugged"""

        self.is_killswitch_on = False
        self.navigation.stop()
