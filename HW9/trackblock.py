import cv2
import imutils
import numpy as np
image = cv2.imread("hw9_red.jpg")
img_resize = imutils.resize(image, width=1000) 
cv2.imshow("caption", img_resize)
a = image.shape
print(a)
img = np.zeros((a[0],a[1],1), dtype=np.uint8)
# cv2.imshow("black", img)
image = cv2.cvtColor(img_resize, cv2.COLOR_BGR2HSV)
lower = np.array([133, 150, 160])
upper = np.array([255, 255, 255])
img = cv2.inRange(image, lower, upper)
cv2.imshow("mask", img)
contour, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
contour = contour[0]
(x,y),rad = cv2.minEnclosingCircle(contour) 
  
center = (int(x),int(y)) 
radius = int(rad) 
cv2.circle(img_resize, center, radius, (0,0,255), 2) 
cv2.imshow("contour", img_resize)
cv2.waitKey(0)
