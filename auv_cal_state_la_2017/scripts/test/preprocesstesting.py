import time
import utils
import glob
import random
import cv2
import numpy as np
import DiceClassifier as dc

dc = dc.DiceClassifier()
hog = dc.get_hog()
lsvm =  dc.get_lsvm()

lower = [0, 80, 80]
upper = [170, 255, 255]
roi_size = 600
test_images = []

def nothing(x):
    pass

for img in glob.glob('dice_test_data/*.jpg'):
    test_images.append(img)

rn = random.randint(0,len(test_images)) - 1

cv2.namedWindow('frame')
cv2.createTrackbar('RL','frame',0,255,nothing)
cv2.createTrackbar('RU','frame',0,255,nothing)
cv2.createTrackbar('GL','frame',0,255,nothing)
cv2.createTrackbar('GU','frame',0,255,nothing)
cv2.createTrackbar('BL','frame',0,255,nothing)
cv2.createTrackbar('BU','frame',0,255,nothing)

def preprocess(lower,upper,frame):
    imhsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    imhsv_blur = cv2.medianBlur(imhsv, 11)
    im_blur = cv2.medianBlur(frame, 25)

    hsv_lower = np.array([40, 40, 40])
    hsv_upper = np.array([100, 255, 255])

    hsv_mask = cv2.inRange(imhsv_blur, hsv_lower, hsv_upper)
    hsv_filter = cv2.bitwise_and(imhsv_blur, imhsv_blur, mask=hsv_mask)

    lower = np.array(lower, dtype='uint8')
    upper = np.array(upper, dtype='uint8')
    mask = cv2.inRange(frame, lower, upper)

    output = cv2.bitwise_and(frame, frame, mask=mask)

    return output, mask, imhsv, imhsv_blur, hsv_filter

while True:

    frame = cv2.imread(test_images[rn])
    clone = frame.copy()
    height, width, lines = frame.shape

    rl = cv2.getTrackbarPos('RL','frame')
    ru = cv2.getTrackbarPos('RU','frame')
    bl = cv2.getTrackbarPos('BL','frame')
    bu = cv2.getTrackbarPos('BU','frame')
    gl = cv2.getTrackbarPos('GL','frame')
    gu = cv2.getTrackbarPos('GU','frame')

    lower = [bl,gl,rl]
    upper = [bu,gu,ru]

    center = (width / 2, height / 2)
    pimage, mask, hsv, hsv_blur, hsv_filter = preprocess(lower,upper,frame)
    imgray = cv2.cvtColor(pimage, cv2.COLOR_BGR2GRAY)
    flag, binary_image = cv2.threshold(imgray, 85, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    edges = cv2.Canny(binary_image, 50, 150)
    im, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.imshow('pimage',pimage)
    # cv2.imshow('mask',mask)
    boxes = [cv2.boundingRect(c) for c in contours]
    interest_regions = [b for b in boxes if b[2] * b[3] > roi_size and b[2] > 50 and b[3] > 50]
    die = dc.classify(frame, interest_regions)
    for x,y,w,h  in interest_regions:
        cv2.rectangle(clone, (x,y),(x+w,y+h),utils.colors["blue"],3)
    if die != None:
        cv2.rectangle(clone, (die[0], die[1]), (die[0] + die[2],  die[1] + die[3]), utils.colors["red"], 3)
        '''
        portion = frame[die[1]:die[1] + die[3], die[0]: die[0] + die[2]]
        cv2.imshow("clone",clone)
        
        choice = input("Enter 1 for false positive 2 for positive 3 to ignore (partial detection)")
        append = time.time()
        print die

        if choice == 1:
            cv2.imwrite("dice_false_pos/" + str(append)+"fp.jpg", portion)
        if choice == 2:
            cv2.imwrite(str(append)+"fp.jpg", portion)
        '''
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('clone',clone)


    '''
    cv2.imshow('pimage', pimage)
    cv2.imshow('hsv_blur', hsv_blur)
    cv2.imshow('hsv_filter', hsv_filtejkkkkr)
    cv2.imshow('mask', mask)
    '''
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
