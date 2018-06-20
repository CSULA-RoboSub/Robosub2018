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
    def bail_task(self, **k): pass

    @abstractmethod
    def restart_task(self, **k): pass
