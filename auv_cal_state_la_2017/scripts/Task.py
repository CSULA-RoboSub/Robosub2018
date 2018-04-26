from abc import ABCMeta, abstractmethod


class Classifier:
    __metaclass__ = ABCMeta

    @abstractmethod
    def is_task_complete(self):
        raise NotImplementedError

    def get_directions(self):
        raise NotImplementedError
