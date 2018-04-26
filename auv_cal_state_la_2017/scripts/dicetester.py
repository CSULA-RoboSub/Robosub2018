import utils
import random
import cv2
import glob
import DiceClassifier
import DicePreprocess as dp

classifier = DiceClassifier.DiceClassifier()
preprocess = dp.DicePreprocessor()

hog = classifier.get_hog()
lsvm = classifier.get_lsvm()
test_images = []

for img in glob.glob('dice_test_data/*.jpg'):
    image =  cv2.imread(img)
    test_images.append(image)

rn = random.randint(0,len(test_images)) - 1
print rn, len(test_images)
print type(test_images[rn])

#dice_pic = test_images[rn]
dice_pic = cv2.imread('dice_test_data/dice1521499757040.jpg')
rois = preprocess.get_interest_regions(dice_pic)

for x, y, w, h in rois:
    cv2.rectangle(dice_pic, (x,y), (x+w, y+h), utils.colors["red"], 2)
while True:
    cv2.imshow('picture', dice_pic )
    preprocess.show_preprocess_images(dice_pic)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break