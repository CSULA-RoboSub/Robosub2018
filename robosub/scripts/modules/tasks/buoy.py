#from modules.sensors.computer_vision import BuoyDetector
from task import Task

class Buoy(Task):
    
    def __init__(self, Houston):
        """ To initialize Buoy """
        super(Buoy, self).__init__()

        self.houston = Houston
        
        self.detectbuoy = None
        self.coordinates = []
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        self.not_found_timer = 0
        self.found_timer = 0

    def detect(self, frame):
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
    
    def navigate(self, navigation, found, coordinates, power, rotation):
        pass
    
    def complete(self):
        pass

    def bail_task(self):
        pass

    def restart_task(self):
        pass
        
    def search(self):
        pass

    def start(self):
        pass
    
    def stop(self):
        pass
        
    def run_detect_for_task(self):
        pass

    def reset_thread(self):
        pass

    def get_most_occur_coordinates(self): 
        pass 
