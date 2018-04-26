import math
import data_utils
import utils
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import DiceClassifier as DC
from matplotlib.widgets import Button
import time
import glob
#frame = data_utils.get_random_die()


def save_pos(win):
    cv2.imwrite('pos_dice/' + str(time.time()) + 'dice.jpg',win)


def save_neg(win):
    cv2.imwrite('neg_images/' + str(time.time()) + 'dice.jpg',win)


frames = []

for img in glob.glob(('dice_test_data/*.jpg')):
    frames.append(cv2.imread(img))
print len(frames)

for frame in frames:
    clone = frame.copy()

    classifier = DC.DiceClassifier()
    lab = cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)

    l,a,b = cv2.split(lab)
    clahe = cv2.createCLAHE(3.0, (8,8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl,a,b))

    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    gray = cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    gedges = cv2.Canny(gray,100,200)
    edges = cv2.Canny(blurred,100,200)

    gim, gcontours, _ = cv2.findContours(gedges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    gboxes = [cv2.boundingRect(c) for c in gcontours]

    grois = [b for b in gboxes if b[2] * b[3] > 400 and math.fabs(b[2] - b[3]) < 30]

    '''
    for x,y,w,h in grois:
        cv2.rectangle(frame, (x,y),(x+w,y+h),utils.colors["green"], 1)
    '''
    dice = classifier.classify(frame,grois)

    for x,y,w,h in dice:
        cv2.rectangle(frame, (x,y),(x+w,y+h),utils.colors["magenta"], 2)
    plt.imshow(frame)
    plt.show()

    print len(dice)

    for x,y,w,h in dice:
        portion = clone[y:y+h,x:x+w]
        plt.imshow(portion)
        plt.show()

        choice = raw_input('is this a false positive? [y/n]')

        if choice == 'y':
            save_neg(portion)

        if choice == 'n':
            save_pos(portion)

        if choice == 's':
            break