from abc import ABCMeta, abstractmethod


class Classifier:
    __metaclass__ = ABCMeta

    @abstractmethod
    def classify(self):
        raise NotImplementedError
