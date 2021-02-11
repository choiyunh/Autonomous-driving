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
    for circle in circles[0, :]:  # Draw all circles
        cv2.circle(image, (circle[0], circle[1]), circle[2], (255, 0, 0), 2)

    # print(imgray[54, 98])  # 녹색 신호 자리
    # print(imgray[54, 68])  # 황색 신호 자리
    # print(imgray[54, 36])  # 적색 신호 자리

    # print(imgray[circles[0][0][1], circles[0][0][0]])

    if len(circles[0]) >= 3:
        lightX = [circles[0][0][0], circles[0][1][0], circles[0][2][0]]
        lightX.sort()
        print(lightX)

        if (lightX[0] + lightX[1] + lightX[2] + 2) // 3 == [lightX[1]]:
            print('This is traffic light')

    cv2.imshow('HoughCircle', image)
else:
    print('원을 찾을 수 없음')

lower_red = np.array([0, 0, 120])
upper_red = np.array([100, 100, 255])

red_range = cv2.inRange(image, lower_red, upper_red)

red_result = cv2.bitwise_and(image, image, mask=red_range)

# cv2.imshow("red", red_result)

cv2.waitKey(0)
cv2.destroyAllWindows()
