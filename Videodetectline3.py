import sys
import math
import cv2 as cv
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from numpy import asarray
import PIL
from PIL import Image

def main(argv):
    
    default_file = 'resized.jpg'
    #filename = argv[0] if len(argv) > 0 else default_file
    # Loads an image
    src = cv.imread(default_file, cv.IMREAD_GRAYSCALE)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1 
    
    
    dst = cv.Canny(src, 200, 500, None, 3)
    pixels = dst
    imagetodraw= np.array(pixels)
    imagetosave = Image.fromarray(imagetodraw.astype(np.uint8))
    imagetosave.save('test2.png', 'png')

    dil = cv.imread('test2.png', 0)
    kernel = np.ones((5,5),np.uint8)
    erosion = cv.dilate(dil,kernel,iterations = 1)
    #cv.imshow("erosion", erosion)


    pixel = erosion
    imagetodraw= np.array(pixel)
    imagetosave = Image.fromarray(imagetodraw.astype(np.uint8))
    imagetosave.save('test_thicc2.png', 'png')


    
    basewidth = 300
    blur = Image.open('test_thicc2.png')
    wpercent = (basewidth / float(blur.size[0]))
    hsize = int((float(blur.size[1]) * float(wpercent)))
    blur = blur.resize((basewidth, hsize), Image.ANTIALIAS)
    blur.save('resized_image2.jpg')


    # Copy edges to the images that will display the results in BGR
    #
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    
    return 0
    
if __name__ == "__main__":
    main(sys.argv[1:])