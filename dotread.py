import math

import cv2
import numpy as np

#image = cv2.imread("dots.jpeg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur = cv2.medianBlur(gray, 11)
thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
thresh2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 25, 15)

# Morph open
#kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
#opening = cv2.morphologyEx(thresh2, cv2.MORPH_OPEN, kernel, iterations=3)

detected_circles = cv2.HoughCircles(thresh2,
                                    cv2.HOUGH_GRADIENT, 0.95, 12, param1=40,
                                    param2=20, minRadius=4, maxRadius=70)

if detected_circles is not None:

    # Convert the circle parameters a, b and r to integers.
    detected_circles = np.uint16(np.around(detected_circles))

    for pt in detected_circles[0, :]:

        a, b, r = pt[0], pt[1], pt[2]

        # Draw a small circle (of radius 1) to show the center.
        cv2.circle(image, (a, b), 1, (0, 0, 255), 3)

if detected_circles is not None:

    detected_circles = np.uint16(np.around(detected_circles))
    for index_curr, pt in enumerate(detected_circles[0, :]):
        a_prev = pt[0]
        b_prev = pt[1]

        a, b = 0, 0
        min_dist = 9999999999999
        idx = 999

        for index_o, pt_o in enumerate(detected_circles[0, :]):
            a, b, r = pt_o[0], pt_o[1], pt_o[2]
            dist_a = a-a_prev if a > a_prev else a_prev-a
            dist_b = b-b_prev if b > b_prev else b_prev-b

            dist = math.sqrt(math.pow(dist_a, 2)+math.pow(dist_b, 2))
            if dist < min_dist and dist != 0:
                min_dist = dist
                idx = index_o
                index_curr = index_o
        cv2.line(image, (a, b), (a_prev, b_prev), (0, 255, 0), 9)

cv2.imshow('gray', gray)
#cv2.imshow('blur', blur)
cv2.imshow('thresh', thresh2)
#cv2.imshow('opening', opening)
cv2.imshow('image', image)
cv2.waitKey()
