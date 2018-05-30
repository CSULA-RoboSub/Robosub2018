from modules.sensors.computer_vision import GateDetector
from task import Task

from modules.control.navigation import Navigation

class Gate(Task):
    
    def __init__(self, Houston):
        """ To initialize Gate """
        super(Gate, self).__init__()
        
        self.houston = Houston
        
        self.detectgate = None
        self.coordinates = []
        self.is_found = False
        self.is_detect_done = False
        self.is_navigate_done = False
        self.is_done = False

        self.rotation_direction = 'right'

        self.not_found_timer = 0
        self.found_timer = 0
        self.gate_circle_loc = 0
    
    def detect(self, frame):
        #add frame when testing complete
        if not self.detectgate:
            self.detectgate = GateDetector.GateDetector()

        found, gate_coordinates = self.detectgate.detect(frame)
        ''' add 'and found is True' when gate circle works '''
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                self.not_found_timer += 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True

        return found, gate_coordinates
    
    def navigate(self, found, coordinates):
        #TODO must implement way for AUV to navigate to complete task
        print 'we have now made it to the navigation section of the task'
        #self.navigation.m_nav('power', 'forward', power)
    
    def complete(self):
        if (self.gate_circle_loc < 2*math.pi):
            self.gate_circle_loc += math.pi/100
            x = math.sin(self.gate_circle_loc)
            y = math.cos(self.gate_circle_loc)
            lower_bound = -.33
            upper_bound = .33

            if (x >= upper_bound):
                coord_x = 1
            elif (x < upper_bound and x >= lower_bound):
                coord_x = 0
            elif (x < lower_bound):
                coord_x = -1

            if (y >= upper_bound):
                coord_y = 1
            elif (y < upper_bound and y >= lower_bound):
                coord_y = 0
            elif (y < lower_bound):
                coord_y = -1
        else:
            self.is_gate_done = True
            print('circling gate completed')
            coord_x = 0
            coord_y = 0
        
        return True, [coord_x, coord_y]

    def bail_task(self):
        print 'bail gate'

    def restart_task(self):
        print 'restart gate'