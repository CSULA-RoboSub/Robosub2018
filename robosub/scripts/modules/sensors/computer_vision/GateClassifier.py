import cv2
import glob
import numpy as np
import pandas as pd
from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn.model_selection import train_test_split
import modules.main.config as config # located in our project folder

class GateClassifier:

    def __init__(self):
        self.new_struct_path = 'modules/sensors/computer_vision/' # project folder struct
        self.model_path = self.new_struct_path + 'models/gate/'
        self.positive_image_path = self.new_struct_path + 'data/gate/positive/*.jpg'
        self.negative_image_path = self.new_struct_path + 'data/gate/negative/*.jpg'
        #134sadfasdafdadfasdfadsfasdfasdfasdf  <- WHAT IS THIS? jaklsdjfafvaej
        self.task_model_config_name = "gate_model" # should be able to rename soon
        self.model_name = self.get_model_name('gate', 'model') # helper for config TODO
        
        self.min_dim = 80
        self.block_size = (16, 16)
        self.block_stride = (8, 8)
        self.cell_size = (8, 8)
        self.bins = 9
        self.dims = (80, 80)
        self.hog = cv2.HOGDescriptor(
            self.dims,
            self.block_size,
            self.block_stride,
            self.cell_size,
            self.bins
        )
   
        self.min_prob = .6  # set probability for MODELdown here for convenience
        # self.set_model(self.model_name)

    # returns the model file name as a string from henrys config file - conig file has prenamed
    def get_model_name(self, section, option):
        return config.get_config(section, option)
        
    def set_model(self, task_model_name=None):
        if task_model_name is None: # so later on we can rename? or something have to ask henry
            task_model_name = self.task_model_config_name
        try:
            self.lsvm = joblib.load(self.model_path + task_model_name + ".pkl")
            print("\nLoading Gate model from disk...\n")
        except IOError as e:
            print("IOError: {0}".format(e) )
            print("\nTraining model...")
            self.lsvm = SVC(kernel="linear", C = 0.1, probability=True, random_state=2)
            self.train_lsvm()
            joblib.dump(self.lsvm, self.model_path + task_model_name + ".pkl") # store model object to disk
            print("\nStoring model to location: " + "\"" + self.model_path + "\"\n")
            

    def get_features_with_label(self, img_data, label):
        data = []
        for img in img_data:
            img = cv2.resize(img, self.dims)
            feat = self.hog.compute(img[:, :, :3] )
            data.append((feat, label))
        return data
    

    def train_lsvm(self):
        pos_imgs = []
        neg_imgs = []
        
        for img in glob.glob(self.positive_image_path):
            n = cv2.imread(img)
            pos_imgs.append(n)
            
        for img in glob.glob(self.negative_image_path):
            n = cv2.imread(img)
            neg_imgs.append(n)
        
        positive_data = self.get_features_with_label(pos_imgs, 1)
        negative_data = self.get_features_with_label(neg_imgs, 0)
        
        data = positive_data + negative_data
        
        np.random.shuffle(data) # use np instead
        
        feat, labels = map(list, zip(*data) )
        feat_flat = [x.flatten() for x in feat]
        
        features_df = pd.DataFrame(feat_flat)
        labels_df = pd.Series(labels)
        
        feat_train, feat_test, label_train, label_test = train_test_split(
            features_df,
            labels_df,
            test_size=0.3,
            random_state=2
        )
        self.lsvm.fit(feat_train, label_train)
        

    '''
    this returns the max value for the GATE, (x,y) as topleft corner
    then w,h and width and height respectively.
    '''
    def classify(self, frame, roi): #roi = regions of interest
        gate = None
        max_val = 0
        if self.lsvm is None:
            print 'error lsvm not trained'
        for box in roi:
            x, y, w, h = box
            window = frame[y:y + h, x:x + w, :]
            window_resized = cv2.resize(window, self.dims)
            feat = self.hog.compute(window_resized)
            feat_reshape = feat.reshape(1, -1)
            prob = self.lsvm.predict_proba(feat_reshape)[0]
            # prediction = self.lsvm.predict(feat_reshape)
            gate_class = prob[1] # corresponds to class 1 (positive gate)
            # print('probability: {}'.format(gate_class))
            # if(prediction > 0 and gate_class > self.min_prob and gate_class > max_val):
            if (gate_class > self.min_prob and gate_class > max_val):
                gate = box
        return gate

    def read_config(self):
        # TODO
        self.model_name = config.get_config('gate', 'model')
        self.set_model(self.model_name)
        self.min_prob = config.get_config('gate', 'min_prob')
