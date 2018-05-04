from task import Task

class CashIn(Task):
    
    def __init__(self):
        """ To initialize CashIn """
        super(CashIn, self).__init__()
        
        self.detectcashin = None
        self.coordinates = []
        self.is_cash_in_found = False
        self.is_cash_in_done = False

        self.not_found_timer = 0
        self.found_timer = 0

    def detect(self):
        print('detect_dice')
        if not self.detectcashin:
            #self.detectcashin = CashInDetector.CashInDetector()
            pass

        found, gate_coordinates = self.detectcashin.detect()
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                gate_coordinates[0] = 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True
            self.task_num += 1

    
    def navigate(self):
        pass
    
    def complete(self):
        pass