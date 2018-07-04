from task import Task

class Slots(Task):
    
    def __init__(self, Houston):
        """ To initialize Slots """
        super(Slots, self).__init__()

        self.houston = Houston
        
        self.detectslots = None
        self.coordinates = []
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
        
        self.not_found_timer = 0
        self.found_timer = 0

    def detect(self, frame):
        print('detect_dice')
        if not self.detectslots:
            #self.detectslots = SlotsDetector.SlotsDetector()
            pass

        found, gate_coordinates = self.detectslots.detect()
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                gate_coordinates[0] = 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True
            self.task_num += 1

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