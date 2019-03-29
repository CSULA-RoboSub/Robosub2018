# import cv2

# img = cv2.imread("gate.jpg")

# img_y_cr_cb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
# y, cr, cb = cv2.split(img_y_cr_cb)

# # Applying equalize Hist operation on Y channel.
# y_eq = cv2.equalizeHist(y)

# img_y_cr_cb_eq = cv2.merge((y_eq, cr, cb))
# img_rgb_eq = cv2.cvtColor(img_y_cr_cb_eq, cv2.COLOR_YCR_CB2BGR)

# cv2.imshow('img_rgb_eq',img_rgb_eq)
# cv2.waitKey(0)

#---------------------------------------------------

import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('gate.jpg',0)

hist,bins = np.histogram(img.flatten(),256,[0,256])

cdf = hist.cumsum()
cdf_normalized = cdf * hist.max()/ cdf.max()

plt.plot(cdf_normalized, color = 'b')
plt.hist(img.flatten(),256,[0,256], color = 'r')
plt.xlim([0,256])
plt.legend(('cdf','histogram'), loc = 'upper left')
# plt.show()

cdf_m = np.ma.masked_equal(cdf,0)
cdf_m = (cdf_m - cdf_m.min())*255/(cdf_m.max()-cdf_m.min())
cdf = np.ma.filled(cdf_m,0).astype('uint8')
img2 = cdf[img]

equ = cv2.equalizeHist(img2)
res = np.hstack((img,equ))

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
cl1 = clahe.apply(res)

cv2.imshow('img2_cli',cl1)
cv2.waitKey(0)

#---------------------------------------------------

# import cv2
# import numpy as np

# img = cv2.imread("gate.jpg")

# equ = cv2.equalizeHist(img)
# res = np.hstack((img,equ))

# cv2.imshow('img_res',res)
# cv2.waitKey(0)