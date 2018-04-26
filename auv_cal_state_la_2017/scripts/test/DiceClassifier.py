import cv2
import glob
import numpy as np
import utils as ut
from sklearn import svm
from random import shuffle
import Classifier

class DiceClassifier:

    def __init__(self):
        self.minDim = 80
        self.blockSize = (16,16)
        self.blockStride = (8,8)
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
        self.lsvm = self.get_lsvm()

    '''note that the height and widths must be multiples of 8 in order to use a HOOG'''
    def get_hog(self):
        return cv2.HOGDescriptor(self.dims, self.blockSize, self.blockStride, self.cellSize, self.nbins, self.derivAperture, self.winSigma, self.histogramNormType, self.L2HysThreshold, self.gammaCorrection, self.nlevels)

    def get_lsvm(self):
        pos_imgs = []
        neg_imgs = []

        for img in glob.glob('./pos_dice/*.jpg'):
            n = cv2.imread(img)
            resized = cv2.resize(n, self.dims)
            pos_imgs.append(resized)

        for img in glob.glob('./neg_images/*.jpg'):
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

        print lsvm.score(train_feat, train_label)
        result = lsvm.predict(test_feat)
        print "test accuracy ", lsvm.score(test_feat, test_label)

        return lsvm

    def classify(self,frame,roi): #roi = regions of interest
        die = None
        max = 0
        true_dice = []
        for box in roi:
            x, y, w, h = box
            window = frame[y:y+h, x:x+w, :]
            window = cv2.resize(window, self.dims)
            feat = self.hog.compute(window)
            prob = self.lsvm.predict_proba(feat.reshape(1, -1))[0]
            if prob[1] > .1 :
                die = box
                true_dice.append(box)
        return true_dice