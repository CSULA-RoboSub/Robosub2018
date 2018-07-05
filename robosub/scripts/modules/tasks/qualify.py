from modules.sensors.computer_vision import GateDetector
from modules.controller.cv_controller import CVController
from task import Task
import time

class Qualify(Task):

    def __init__(self, Houston):
        super(Qualify, self).__init__()
        self.houston = Houston
        self.is_done = False
        self.object_detected = False
        self.been_detected = False
        self.detector = GateDetector.GateDetector()
        self.search_time_lim =  120 #2 minutes
        '''
            the phases are:
            1, finding the gate the first time;
            2, going around the pole;
            3, finding the gate the second time;
            4, the gate has been completed.
        '''
        self.phase = 1 # phase should be either 1, 2, 3, 4   self.vertical_movement = {-1: 'down', 0: 'staying', 1: 'up'}
        self.rotation_movement = {-1: 'left', 0: 'staying', 1: 'right'}
        self.rotation_angle = 10


    def navigate(self, navi, found, dirs, power, rotation):

        navi.cancel_r_nav()
        navi.r_nav(self.rotation_movement[dirs[0]]
                   , rotation, power)

        navi.cancel_m_nav()
        navi.m_nav('power', 'forward', power)

    def search(self, frame, navi, prev_detected, rot, pow):
        '''
            Span_limit are is the number of seconds to
            scan per direction.
        '''
        span_limit = 0

        if prev_detected:
            span_limit = 3
        else:
            span_limit = 5

        start_time =  time.time()
        direction = 'left'
        found = False
        coords = [0,0]

        while time.time() - start_time < self.search_time_lim\
                and not found:

            navi.cancel_m_nav()

            navi.cancel_r_nav()
            navi.r_nav(direction, rot, pow)

            found, coords = self.detect(frame)

            if (time.time() - start_time) % span_limit // 1 == 0:

                if direction == 'left':
                    direction = 'right'
                else:
                    direction = 'left'

        return found, coords

    def complete(self):
        if self.phase == 4:
            self.is_done = True

    def bail_task(self): pass

    def restart_task(self): pass

    
    def search(self):
        pass

    def detect(self, frame):
        return self.detector.detect(frame)
