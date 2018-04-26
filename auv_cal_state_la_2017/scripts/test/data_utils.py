import cv2
import glob
import random


def get_random_die():
    imgs = []
    for img in glob.glob('dice_test_data/*.jpg'):
        imgs.append(cv2.imread(img))

    return imgs[random.randint(0, len(imgs)-1)]
