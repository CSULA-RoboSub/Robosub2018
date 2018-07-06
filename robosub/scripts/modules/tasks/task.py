from abc import ABCMeta, abstractmethod

class Task(object):

    __metaclass__ = ABCMeta

    @abstractmethod
    def detect(self, frame): pass

    @abstractmethod
    def navigate(self, **k): pass

    @abstractmethod
    def search(self, **k): pass

    @abstractmethod
    def complete(self, **k): pass

    @abstractmethod
    def restart_task(self): pass

    @abstractmethod
    def reset(self): pass

    @abstractmethod
    def start(self): pass

    @abstractmethod
    def stop(self): pass

    @abstractmethod
    def run_detect_for_task(self): pass

    @abstractmethod
    def reset_thread(self): pass

    @abstractmethod
    def get_most_occur_coordinates(self): pass 
