import numpy as np
import cv2 as cv
cap = cv.VideoCapture(-1)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here

    #added for noise reduction
    kernel_size = (3,3)
    gauss_image = cv.GaussianBlur(frame, kernel_size, 0)
    lower_black = np.array([0, 0, 0,])
    upper_black = np.array([227,100,70])
    hsv_image = cv.cvtColor(gauss_image, cv.COLOR_BGR2HSV)
    thres_1 = cv.inRange(hsv_image, lower_black, upper_black)
    low_threshold = 200
    high_threshold = 400
    canny_edges = cv.Canny(thres_1, low_threshold, high_threshold)
    def region_of_interest(edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)

        polygon = np.array([[
         (0, height * 1/2 ),
         (width, height *1/2),
         (width, height),
         (0, height),
        ]], np.int32)

        cv.fillPoly(mask, polygon, 255)
        cropped_edges = cv.bitwise_and(edges, mask)
        return cropped_edges

    roi_image = region_of_interest(canny_edges)

    def detect_line_segments(cropped_edges):
        rho = 1
        angle = np.pi/180
        min_threshold = 10
        line_segments = cv.HoughLinesP(cropped_edges, rho, angle,
           min_threshold, np.array([]), minLineLength=10, maxLineGao=15)
        return line_segments
    line_segments = detect_line_segments(roi_image)
    
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()