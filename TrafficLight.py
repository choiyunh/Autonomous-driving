import numpy as np
import cv2

def light_detection_3(frame):
    global limit
    frame = frame[160:230, 180:480]
    cv2.imshow('frame', frame)
    frame2 = frame.copy()
    frame2 = cv2.GaussianBlur(frame2, (9, 9), 0)
    imgray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(imgray, cv2.HOUGH_GRADIENT, 1, 10, param1=60, param2=25, minRadius=11, maxRadius=50)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            cv2.circle(frame, (circle[0], circle[1]), circle[2], (255, 0, 0), 2)
            cv2.imshow('frame', frame)
        if len(circles[0]) == 3:
            lightX = [circles[0][0][0], circles[0][1][0], circles[0][2][0]]
            lightY = [circles[0][0][1], circles[0][1][1], circles[0][2][1]]
            lightX.sort()
            lightY.sort()

            if (lightX[0] + lightX[1] + lightX[2] + 2) // 3 == lightX[1]:
                print('COLOR VALUE : ', imgray[lightY[2], lightX[2]])
                if imgray[lightY[2], lightX[2]] > 230:  # light on
                    return True
                return False
    return True

def light_detection_4(frame):
    global limit
    frame = frame[180:250, 200:460]

    frame2 = frame.copy()
    frame2 = cv2.GaussianBlur(frame2, (9, 9), 0)
    imgray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(imgray, cv2.HOUGH_GRADIENT, 1, 10, param1=60, param2=25, minRadius=10, maxRadius=40)

    if circles is not None:
        circles = np.uint16(np.around(circles))

        for circle in circles[0, :]:
            cv2.circle(frame, (circle[0], circle[1]), circle[2], (255, 0, 0), 2)
            cv2.imshow('frame', frame)
        if len(circles[0]) == 4:
            lightX = [circles[0][0][0], circles[0][1][0], circles[0][2][0], circles[0][3][0]]
            lightY = [circles[0][0][1], circles[0][1][1], circles[0][2][1], circles[0][3][1]]
            lightX.sort()
            lightY.sort()

            if (lightX[0] + lightX[3]) == (lightX[1] + lightX[2]):
                if imgray[lightY[0], lightX[0]] >= 230:  # left light
                    limit = 5
                elif imgray[lightY[3], lightX[3]] >= 230:  # right light
                    limit = 6
                return False
    return True

frame = cv2.imread('./img/b.png')
cv2.imshow('original', frame)
if light_detection_3(frame):
    print('circle')
cv2.waitKey(0)
cv2.destroyAllWindows()
