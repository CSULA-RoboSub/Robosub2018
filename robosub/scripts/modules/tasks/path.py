from task import Task

class Path(Task):
    
    def __init__(self, Houston):
        """ To initialize Path """
        super(Path, self).__init__()

        self.houston = Houston
        
        self.detectpath = None
        self.coordinates = []
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False
    
        self.not_found_timer = 0
        self.found_timer = 0

    def detect(self, frame):
        '''if not self.detectpath:
            #self.detectpath = PathDetector.PathDetector()
            pass

        found, gate_coordinates = self.detectpath.detect()
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                gate_coordinates[0] = 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True
            self.task_num += 1'''
        print 'detect path'
        self.navigate()
        self.complete()
        self.bail_task()
        self.restart_task()

        return False, [0,0]


    def navigate(self, navigation, found, coordinates, power, rotation):
        print 'navigate path'
    
    def complete(self):
        print 'complete path'

    def bail_task(self):
        print 'bail task path'

    def restart_task(self):
        print 'restart task path'


    def search(self):
        pass
        
    def start(self):
        self.navigation.start()
    
    def stop(self):
        self.navigation.stop()