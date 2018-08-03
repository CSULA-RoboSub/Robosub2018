from abc import ABCMeta, abstractmethod


class Preprocessor:
    __metaclass__ = ABCMeta

    @abstractmethod
    def preprocess(self):
        raise NotImplementedError
