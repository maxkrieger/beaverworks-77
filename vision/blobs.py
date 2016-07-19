# Standard imports
import cv2
import numpy as np;

# Read image
im = cv2.imread("images/frame0070.jpg")

#im = cv2.resize(im, (0,0), fx=0.75, fy=0.75)

out = im.copy()

hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

ranges = [
    [[80, 50, 60], [100, 255, 255]],
    [[145, 0, 100], [165, 255, 255]],
    [[20, 230, 200], [25, 255, 255]],
    [[7, 150, 200], [13, 255, 255]],
    [[55, 100, 60], [65, 255, 255]]
]

for r1, r2 in ranges:
    r1 = np.array(r1)
    r2 = np.array(r2)

    mask = cv2.inRange(hsv, r1, r2)

    mask = cv2.GaussianBlur(mask, (21,21), 0)
    mask = cv2.erode(mask, (5,5), iterations=1)
    contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(out, contours, -1, (0, 255, 0), 2)

    for j in range(0, len(contours)):
        rect = cv2.minAreaRect(contours[j])
        
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)

        mask = np.zeros((im.shape[0], im.shape[1]), np.uint8)
        cv2.drawContours(mask, [box], 0, (255), -1)

        avg_b, avg_g, avg_r, _ = cv2.mean(im, mask=mask)
        avg_b, avg_g, avg_r = int(avg_b), int(avg_g), int(avg_r)
        print(avg_b, avg_g, avg_r)

        cv2.drawContours(out,[box],0,(30,30,30),10)
        cv2.drawContours(out,[box],0,(avg_b,avg_g,avg_r),4)

# Show blobs
cv2.imshow("Yo", out)

cv2.waitKey(0)
