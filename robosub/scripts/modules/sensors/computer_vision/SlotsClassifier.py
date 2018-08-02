import cv2
import glob
import numpy as np
import pandas as pd
from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn.model_selection import train_test_split
import modules.main.config as config # located in our project folder

class SlotsClassifier:

    def __init__(self):
        pass

    def get_model_name(self, section, option):
        return config.get_config(section,option)

    def set_modeil(self, task_model_name=None):
        pass

    def get_features_with_labels(self, img_data, label):
        pass

    def train_lsvm(self):
        pass

    def classify(self, frame, roi):
        slots = None