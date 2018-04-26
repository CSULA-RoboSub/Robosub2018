import matplotlib as mpl
import cv2
import utils
import math
import DiceClassifier as DC

frame = cv2.imread('./dice_test_data/dice1521499707100.jpg')
#frame = cv2.imread('dice_test_data/dice1521499311440.jpg')
#frame = cv2.imread('gate.jpg')
classifier = DC.DiceClassifier()
lab = cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)

l,a,b = cv2.split(lab)
clahe = cv2.createCLAHE(3.0, (8,8))
cl = clahe.apply(l)
limg = cv2.merge((cl,a,b))

final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
gray = cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
blurred =  cv2.GaussianBlur(gray, (5,5), 0)
gedges = cv2.Canny(gray,100,200)
edges = cv2.Canny(blurred,100,200)

im, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
gim, gcontours, _ = cv2.findContours(gedges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

gboxes = [cv2.boundingRect(c) for c in gcontours]
boxes = [cv2.boundingRect(c) for c in contours]

rois = [b for b in boxes if b[2] * b[3] > 400 and math.fabs(b[2] - b[3]) < 30]
grois = [b for b in gboxes if b[2] * b[3] > 400 and math.fabs(b[2] - b[3]) < 30]

print len(rois)
print len(grois),'grois'
'''
for x, y, w, h in rois:
    cv2.rectangle(final, (x, y), (x+w, y+h), utils.colors["blue"], 1)

for x, y, w, h in grois:
    cv2.rectangle(final, (x, y), (x+w, y+h), utils.colors["red"], 1)
dice = classifier.classify(frame,grois)
print len(grois)
print len(dice)

for x, y, w, h in dice:
    cv2.rectangle(final, (x, y), (x+w, y+h), utils.colors["magenta"], 2)
'''
while True:

    cv2.imshow('basic_edges',cv2.Canny(frame,100,200))
    cv2.imshow('edges',edges)
    cv2.imshow('gedges',gedges)
    cv2.imshow('gray',gray)
    cv2.imshow('final',final)
    cv2.imshow('blurred',blurred)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
