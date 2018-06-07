from task import Task

class Chip(Task):
    
    def __init__(self, Houston):
        """ To initialize Pinger A """
        super(Chip, self).__init__()

        self.houston = Houston
        
        self.detectchip = None
        self.coordinates = []
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        self.not_found_timer = 0
        self.found_timer = 0

    def detect(self, frame):
        pass

    def navigate(self, navigation, found, coordinates, power, rotation):
        pass
    
    def complete(self):
        pass

    def bail_task(self):
        pass

    def restart_task(self):
        pass

    def start(self):
        self.navigation.start()
    
    def stop(self):
        self.navigation.stop()