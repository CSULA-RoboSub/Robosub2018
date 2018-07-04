#from modules.sensors.computer_vision import GatePreprocessor
import cv2
import numpy as np
import matplotlib.pyplot as plt

gate = cv2.imread("/home/csula/Pictures/gate/gate0617.jpg")

lower = np.array([0, 100, 50], 'uint8')  # lower color value
upper = np.array([10, 255, 255], 'uint8')

ycbcr = cv2.cvtColor(gate,cv2.COLOR_BGR2YCR_CB)
y, cb, cr = cv2.split(ycbcr)
clahe = cv2.createCLAHE(3.0, (8, 8))
y_equ = clahe.apply(y)
#img = cv2.merge((y_equ, cb, cr))
#final = cv2.cvtColor(img, cv2.COLOR_LAB2BGR)
#cv2.imwrite('clahe_final.jpg',final)
#

mask = cv2.inRange(y_equ, lower, upper)
output = cv2.bitwise_and(y_equ, y_equ, mask=mask)
_, binary_image = cv2.threshold(output, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
edges = cv2.Canny(binary_image, 50, 150)

'''
lines = cv2.HoughLinesP(edges,1,np.pi/180,100)
for x1, y1, x2, y2 in lines[0]:
    cv2.line(gate, (x1, y1), (x2, y2), (0, 255, 0), 2)


lines = cv2.HoughLines(edges,1,np.pi/180,100)
print lines
for line in lines:
    for rho, theta in line:
        print rho, theta
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 100*(-b))
        y1 = int(y0 + 100*(a))
        x2 = int(x0 - 100*(-b))
        y2 = int(y0 - 100*(a))

        cv2.line(gate,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imwrite('houghlines3.jpg',gate)

'''

while True:
    cv2.imshow('gate', gate)
#    cv2.imshow('blur', blur)
    cv2.imshow('luminance', y_equ)
    #cv2.imshow('bin', binary_image)
    cv2.imshow('edges', edges)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()