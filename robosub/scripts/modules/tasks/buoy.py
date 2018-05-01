from modules.sensors.computer_vision import BuoyDetector
from task import Task

class Buoy(Task):
    
    def __init__(self):
        """ To initialize Buoy """
        super(Buoy, self).__init__()
        
        self.detectbuoy = None
        self.coordinates = []
        self.is_buoy_found = False
        self.is_buoy_complete = False

        self.not_found_timer = 0
        self.found_timer = 0

    def detect(self):
        print('detect_buoy')
        if not self.detectbuoy:
            self.detectbuoy = BuoyDetector.BuoyDetector()

        found, gate_coordinates = self.detectbuoy.detect()
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                gate_coordinates[0] = 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True
            self.task_num += 1

        return found, gate_coordinates
    
    def navigate(self):
        pass
    
    def complete(self):
        pass