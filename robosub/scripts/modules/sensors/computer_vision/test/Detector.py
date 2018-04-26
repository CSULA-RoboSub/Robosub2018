from abc import ABCMeta, abstractmethod


class Detector:
    __metaclass__ = ABCMeta

    def __init__(self):
        self.preprocessor
        self.classifier
        self.cap
        self.directions

    @abstractmethod
    def detect(self):
        raise NotImplementedError

    @abstractmethod
    def update_directions(self):
        raise NotImplementedError
