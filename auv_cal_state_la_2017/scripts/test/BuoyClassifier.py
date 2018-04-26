
import cv2
import glob
import numpy as np
import utils as ut
from sklearn import svm
from random import shuffle
import Classifier


class BuoyClassifier:

    def __init__(self):
        self.minDim = 80
        self.blockSize = (16, 16)
        self.blockStride = (8, 8)
        self.cellSize = (8, 8)
        self.nbins = 9
        self.derivAperture = 1
        self.winSigma = -1
        self.histogramNormType = 0
        self.L2HysThreshold = 2.1e-1
        self.gammaCorrection = 0
        self.nlevels = 64
        self.dims = (96, 144)
        self.hog = self.get_hog()
        self.lsvm = self.get_lsvm()
        print 'classifier finished being built'

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

    def get_lsvm(self):
        pos_imgs = []
        neg_imgs = []

        for img in glob.glob('pos_images/*.jpg'):
            n = cv2.imread(img)
            pos_imgs.append(n)

        for img in glob.glob('neg_images/*.jpg'):
            n = cv2.imread(img)
            neg_imgs.append(n)

        positive_data = self.get_features_with_label(pos_imgs, 1)
        negative_data = self.get_features_with_label(neg_imgs, 0)

        data = positive_data + negative_data
        shuffle(data)

        feat, labels = map(list, zip(*data))
        feat = [x.flatten() for x in feat]

        sample_size = len(feat)
        train_size = int(round(0.8*sample_size))

        train_feat = np.array(feat[:train_size], np.float32)
        test_feat = np.array(feat[train_size: sample_size], np.float32)
        train_label = np.array(labels[:train_size])
        test_label = np.array(labels[train_size:sample_size])
        lsvm = svm.SVC(gamma=5, C=.12, kernel="linear", probability=True)
        lsvm.fit(train_feat, train_label)

        result = lsvm.predict(test_feat)

        return lsvm
    '''
        this returns the max value for the buoy, (x,y)  as topleft corner 
        then w,h and width and height respectively.
    '''
    def classify(self,frame,roi): #roi = regions of interest
        buoy = None
        max = 0
        for box in roi:
            x, y, w, h = box
            window = frame[y:y+h, x:x+w, :]
            window = cv2.resize(window, self.dims)
            feat = self.hog.compute(window)
            prob = self.lsvm.predict_proba(feat.reshape(1, -1))[0]
            if prob[1] > .1 and prob[1] > max:
                buoy = box
        return buoy
