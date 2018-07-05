import PlatformPreprocessor as pp
import cv2
import numpy as np

img =  cv2.imread('dice_dimensions.png')
pp = pp.PlatformPreprocessor()
pimg, mask = pp.preprocess(img)
imgray = cv2.cvtColor(pimg, cv2.COLOR_BGR2GRAY)
flag, binary_image = cv2.threshold(imgray, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)


while True:
    cv2.imshow('bin',binary_image)
    cv2.imshow('dice_task', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
