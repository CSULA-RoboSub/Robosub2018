from config.config import Config

class Orientation():
    def __init__(self):
        self.config = Config()
        self.get_orientation()

    def get_orientation(self):
        self.start_orientation = self.config.get_config('auv', 'start_orientation')
        self.start_angle = self.config.get_config('auv', 'start_angle')