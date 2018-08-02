import cv2
import glob
import numpy as np
import pandas as pd
from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn.model_selection import train_test_split
import modules.main.config as config # located in our project folder

class DiceClassifier:

    def __init__(self):
        self.new_struct_path = 'modules/sensors/computer_vision/' # project folder struct
        self.model_path = self.new_struct_path + 'models/dice/'
        self.positive_image_path = self.new_struct_path + 'data/dice/positive/*.jpg'
        self.negative_image_path = self.new_struct_path + 'data/dice/negative/*.jpg'
        self.task_model_config_name = "DiceSVMstd"
        self.model_name = self.get_model_name('dice', 'model')  # TODO
        # will convert hog to same way gate is 
        self.minDim = 80
        self.blockSize = (16, 16)
        self.blockStride = (8, 8)
        self.cellSize = (8,8)
        self.nbins = 9
        self.derivAperture = 1
        self.winSigma = -1
        self.histogramNormType = 0
        self.L2HysThreshold = 2.1e-1
        self.gammaCorrection = 0
        self.nlevels = 64
        self.dims = (144,144)
        self.hog = self.get_hog()
        
        self.min_prob = .9 # adjust probability here
        # self.set_model(self.model_name)


    # returns the model file name as a string from henry's config file
    def get_model_name(self, section, option):
        return config.get_config(section, option)


    def set_model(self, task_model_name=None):
        if task_model_name is None:
            task_model_name = self.task_model_config_name
        try:
            self.lsvm = joblib.load(self.model_path + task_model_name + ".pkl")
            print("\nLoading DICE model from disk...\n")
        except IOError as e:
            print("IOError: {0}".format(e) )
            print("\nTraining model...")
            self.lsvm = SVC(gamma=5, C=.5 , kernel="linear", probability=True, random_state=2)
            self.train_lsvm()
            joblib.dump(self.lsvm, self.model_path + task_model_name + ".pkl")
            print("\nStoring model to location: " + "\"" + self.model_path + "\"\n'")

    '''note that the height and widths must be multiples of 8 in order to use a HOOG'''
    def get_hog(self):
        return cv2.HOGDescriptor(self.dims, self.blockSize, self.blockStride, self.cellSize, self.nbins, self.derivAperture, self.winSigma, self.histogramNormType, self.L2HysThreshold, self.gammaCorrection, self.nlevels)


    def get_features_with_label(self, img_data, label):
        data = []
        for img in img_data:
            img = cv2.resize(img, self.dims)
            feat = self.hog.compute(img[:, :, :3])
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

   
    def classify(self, frame, roi): #roi = regions of interest
        die = None
        max_val = 0
        if self.lsvm is None:
            print 'ERROR: lsvm not trained'

        classified_rois = []
        for box in roi:
            x, y, w, h = box
            window = frame[y:y + h, x:x + w, :]
            window_resized = cv2.resize(window, self.dims)
            feat = self.hog.compute(window_resized)
            feat_reshape = feat.reshape(1, -1)
            prob = self.lsvm.predict_proba(feat_reshape)[0]
            # prediction = self.lsvm.predict(feat_reshape)
            dice_class = prob[1]
            # print(dice_class)
            # if (prediction > 0 and dice_class >= self.min_prob and dice_class > max_val):
            if (dice_class >= self.min_prob and dice_class > max_val):
                classified_rois.append(box)
        return classified_rois

    
    def read_config(self):
        # TODO
        self.model_name = config.get_config('dice', 'model')
        self.set_model(self.model_name)
        self.min_prob = config.get_config('dice', 'min_prob')
