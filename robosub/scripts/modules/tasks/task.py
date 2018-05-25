from abc import ABCMeta, abstractmethod

class Task(object):

    __metaclass__ = ABCMeta

    @abstractmethod
    def detect(self): pass

    @abstractmethod
    def navigate(self): pass

    @abstractmethod
    def complete(self): pass

    @abstractmethod
    def bail_task(self): pass

    @abstractmethod
    def restart_task(self): pass