import modules.main.config as config

class Orientation():
    def __init__(self):
        self.get_orientation()
        self.is_orientation_ran = False

    def reset(self):
        self.is_orientation_ran = False

    def get_orientation(self):
        self.start_orientation = config.get_config('auv', 'start_orientation')
        self.start_angle = config.get_config('auv', 'start_angle')

    def set_orientation(self, navigation, power, r_power):
        if not self.is_orientation_ran:
            self.is_orientation_ran = True
            navigation.r_nav(self.start_orientation, self.start_angle, r_power)
            navigation.ros_sleep(3)
            navigation.m_nav('power', 'forward', power)
            navigation.ros_sleep(3)
