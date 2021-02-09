import numpy as np
import cv2

image = cv2.imread('./img/b_red.png')
cv2.imshow("original", image)
image = image[220:280, 250:380]

image2 = image.copy()
image2 = cv2.GaussianBlur(image2, (9, 9), 0)
imgray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

circles = cv2.HoughCircles(imgray, cv2.HOUGH_GRADIENT, 1, 10, param1=60, param2=30, minRadius=0, maxRadius=75)

if circles is not None:
    circles = np.uint16(np.around(circles))
    print(circles)
    print(len(circles[0]))
    for i in circles[0, :]:
        cv2.circle(image, (i[0], i[1]), i[2], (255, 0, 0), 2)

    cv2.imshow('HoughCircle', image)
else:
    print('원을 찾을 수 없음')

lower_red = np.array([0, 0, 120])
upper_red = np.array([100, 100, 255])

red_range = cv2.inRange(image, lower_red, upper_red)

red_result = cv2.bitwise_and(image, image, mask=red_range)

#cv2.imshow("red", red_result)

cv2.waitKey(0)
cv2.destroyAllWindows()
