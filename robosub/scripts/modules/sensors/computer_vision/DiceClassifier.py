import cv2
import glob
import numpy as np
import utils as ut
import pickle
from sklearn import svm
from sklearn.externals import joblib
from random import shuffle
import Classifier

class DiceClassifier:

    def __init__(self):
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
        self.min_prob = .1 # adjust probability here
        self.hog = self.get_hog()
        self.lsvm = self.get_lsvm()
        
        self.confidence_threshold = .1

    '''note that the height and widths must be multiples of 8 in order to use a HOOG'''
    def get_hog(self):
        return cv2.HOGDescriptor(self.dims, self.blockSize, self.blockStride, self.cellSize, self.nbins, self.derivAperture, self.winSigma, self.histogramNormType, self.L2HysThreshold, self.gammaCorrection, self.nlevels)
    '''
        Checks if there is a pickle svm already if not it'll create it.
    '''
    def get_lsvm(self):

        lsvm = None

        try:
            lsvm = joblib.load('modules/sensors/computer_vision/models/dice/DiceSVMstd.pkl')
            print("\nLoading Dice model from disk...\n")
        except IOError:
            print("SVM not found \n Building SVM")
            pos_imgs = []
            neg_imgs = []

            for img in glob.glob('modules/sensors/computer_vision/data/dice/pos_dice/*.jpg'):
                n = cv2.imread(img)
                resized = cv2.resize(n, self.dims)
                pos_imgs.append(resized)

            for img in glob.glob('modules/sensors/computer_vision/data/dice/neg_images/*.jpg'):
                n = cv2.imread(img)
                neg_imgs.append(n)

            def getFeaturesWithLabel(imgData, hog, dims, label):
                data = []
                for img in imgData:
                    img = cv2.resize(img, dims)
                    #for images with transparency layer, reduce to 3 layers
                    feat = hog.compute(img[:,:,:3])
                    data.append((feat, label))
                return data

            pdata = getFeaturesWithLabel(pos_imgs, self.hog, self.dims, 1)
            ndata = getFeaturesWithLabel(neg_imgs, self.hog, self.dims, 0)

            data = pdata + ndata
            shuffle(data)

            feat, labels =  map(list, zip(*data))
            feat = [x.flatten() for x in feat]

            sample_size = len(feat)
            train_size = int(round(0.8*sample_size))

            train_feat = np.array(feat[:train_size], np.float32)
            test_feat = np.array(feat[train_size: sample_size], np.float32)
            train_label = np.array(labels[:train_size])
            test_label = np.array(labels[train_size:sample_size])
            
            lsvm = svm.SVC(gamma=5, C= .5 , kernel="linear", probability=True)
            lsvm.fit(train_feat, train_label)

            joblib.dump(lsvm,'modules/sensors/computer_vision/models/dice/DiceSVMstd.pkl')
            result = lsvm.predict(test_feat)

            #print "test accuracy ", lsvm.score(test_feat, test_label)
            #print lsvm.score(train_feat, train_label)

        return lsvm
    
    def set_confidence_thresh(self, num):
        self.confidence_threshold = num
   
    def classify(self, frame, roi): #roi = regions of interest
        die = None
        max_val = 0
        if self.lsvm is None:
            print 'ERROR: lsvm not trained'
        for box in roi:
            x, y, w, h = box
            window = frame[y:y + h, x:x + w, :]
            window_resized = cv2.resize(window, self.dims)
            feat = self.hog.compute(window_resized)
            feat_reshape = feat.reshape(1, -1)
            prob = self.lsvm.predict_proba(feat_reshape)[0]
            prediction = self.lsvm.predict(feat_reshape)
            dice_class = prob[1]
            if (prediction > 0 and gate_class >= self.min_prob and gate_class > max_val):
                die = box
        return die
