import cv2
import matplotlib.pyplot as plt
import numpy as np

img = cv2.imread('resized_image2.jpg') # read an image from a file using
cv2.circle(img,(270,21), 1, (255,0,0), -1) # add a circle at (5, 220)
cv2.circle(img, (163,72), 1, (0,0,255), -1) # add a circle at (5,5)
plt.figure(figsize=(7,7))
plt.imshow(img) # show the image
plt.show()