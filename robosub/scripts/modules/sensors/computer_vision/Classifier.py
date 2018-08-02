from abc import ABCMeta, abstractmethod
import modules.main.config as config # located in our project folder


class Classifier:
    __metaclass__ = ABCMeta

    @abstractmethod
    def classify(self):
        raise NotImplementedError
