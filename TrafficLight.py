import numpy as np
import cv2

image = cv2.imread('./img/b.png')
cv2.imshow("original", image)
image = image[150:350, 250:380]

image2 = image.copy()
image2 = cv2.GaussianBlur(image2, (9, 9), 0)
imgray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

circles = cv2.HoughCircles(imgray, cv2.HOUGH_GRADIENT, 1, 10, param1=60, param2=30, minRadius=0, maxRadius=60)
cv2.imshow("imgray", imgray)
if circles is not None:
    circles = np.uint16(np.around(circles))
    print(circles)
    for circle in circles[0, :]:
        cv2.circle(image, (circle[0], circle[1]), circle[2], (255, 0, 0), 2)

    if len(circles[0]) == 3:
        lightX = [circles[0][0][0], circles[0][1][0], circles[0][2][0]]
        lightX.sort()

        if (lightX[0] + lightX[1] + lightX[2] + 2) // 3 == lightX[1]:
            if imgray[circles[0][2][1], circle[0][2][0]] <= 160:  # light on
                print('green light on')
            print('This is traffic light')
    elif len(circles[0]) == 4:
        lightX = [circles[0][0][0], circles[0][1][0], circles[0][2][0]]
        lightX.sort()

        if (lightX[0] + lightX[1] + lightX[2] + 2) // 3 == lightX[1]:
            if imgray[circles[0][0][1], circle[0][0][0]] <= 160:  # left light
                limit = 5
            if imgray[circles[0][3][1], circle[0][3][0]] <= 160:  # right light
                limit = 6
            print('This is traffic light')

else:
    print('원을 찾을 수 없음')


cv2.waitKey(0)
cv2.destroyAllWindows()
