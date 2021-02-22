#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math, time
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image, LaserScan
from detect_stopline import detect_stoplineB

import sys
import os
import signal


def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

lidar_range = []
image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 400
Gap = 40
flag = 0
limit = 0
c = 0
mission = 0


def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")


def lidar_callback(data):
    global lidar_range
    lidar_range = data.ranges


# publish xycar_motor msg
def drive(Angle, Speed):
    global pub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)


# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1 + Offset), (x2, y2 + Offset), color, 2)
    return img


# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                  (lpos + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                  (rpos + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (center - 5, 15 + offset),
                  (center + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (315, 15 + offset),
                  (325, 25 + offset),
                  (0, 0, 255), 2)
    return img


# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2 - y1) / float(x2 - x1)

        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width / 2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width / 2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines


# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b


# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height / 2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height / 2)), (255, 0, 0), 3)

    return img, int(pos)


# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap

    # gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    roi = edge_img[Offset: Offset + Gap, 0: Width]
    all_lines = cv2.HoughLinesP(roi, 1, math.pi / 180, 30, 30, 10)

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255, 255, 255), 2)

    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    # roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    # roi2 = draw_rectangle(roi2, lpos, rpos)

    # show image
    cv2.imshow('calibration', frame)

    return lpos, rpos


def light_detection_3(frame):
    global limit
    frame = frame[150:220, 210:450]
    cv2.imshow('frame', frame)
    frame2 = frame.copy()
    frame2 = cv2.GaussianBlur(frame2, (9, 9), 0)
    imgray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(imgray, cv2.HOUGH_GRADIENT, 1, 10, param1=60, param2=25, minRadius=12, maxRadius=60)

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

def scan_front(degree_s, degree_e, dist):
    global lidar_range
    flag_ = False
    for degree in range(degree_s, degree_e):	# scan degree
        if lidar_range[degree] <= dist:
            print(lidar_range[degree], degree)
            flag_ = True
    return flag_
  
def avoid_obstacles():
    global limit, lidar_range, flag, c		# left = 0', right = 180'

    drive_time = time.time() + 2
    while True:
        drive(0, 3)
        print('111111111111')
        if time.time() > drive_time:
            break
   
    drive_time = time.time() + 1.5
    while True:
        drive(-75, 2)
        print('2222222222222')
        if time.time() > drive_time:
            break
    
    drive_time = time.time() + 3.5
    while True:
        drive(50, 2)
        print('333333333333333')
        if time.time() > drive_time:
            break
    
    drive_time = time.time() + 2
    while True:
        drive(-7, 2)
        print('444444444444444')
        if time.time() > drive_time:
            break

def avoid_obstaclesB():
    global mission
    drive_time = time.time() + 3
    print('1111111111111')
    while True:
        drive(-15, 3)
        if time.time() > drive_time:
            break
    
    drive_time = time.time() + 4
    print('222222222222222')
    while True:
        drive(25, 3)
        if time.time() > drive_time:
            break
    
    drive_time = time.time() + 3
    print('33333333333333')
    while True:
        drive(-10, 3)
        if time.time() > drive_time:
            break
    print('-----------Change mission 4----------------')
    mission = 4

def entry_rotary(mission_ = 2, dTime_ = 2.5):
    global mission
    rate = rospy.Rate(10)
    drive_time = time.time() + dTime_
    while True:
        if mission_ == 1:
            drive(0,5)
        else:
            drive(8, 4)
        rate.sleep()
        if time.time() > drive_time:
            break
    # change mission
    print('--------------Change mission ', mission_, '---------------------')
    mission = mission_

def turn_left():
    global mission
    
    drive_time = time.time() + 3.5
    while True:
        drive(-15, 3)
        if time.time() > drive_time:
            break
    mission = 6

def turn_right():
    global mission
    
    drive_time = time.time() + 3
    while True:
        drive(15, 3)
        if time.time() > drive_time:
            break
    drive_time = time.time() + 2.5
    while True:
        drive(70, 3)
        if time.time() > drive_time:
            break
    mission = 6

def light_detection_4(frame):
    global limit, mission
    frame = frame[180:250, 210:450]

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
                if imgray[lightY[0], lightX[0]] >= 220:  # left light
                    turn_left()
                    print('left')
                elif imgray[lightY[3], lightX[3]] >= 220:  # right light
                    turn_right()
                    print('right')
                return False
    return True

    
def start():
    global pub
    global image
    global Gap
    global Width, Height
    global c, mission

    pid_P = 0.7
    pid_I = 0
    pid_D = 0
    sum_angle = 0
    mission = 0
    cnt = 0
    prev_angle = 0

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

    print("---------- Xycar A2 v1.1.0 ----------")
    rospy.sleep(4)
    
    rate = rospy.Rate(10)
    print('START!!!!')
    mission_start = 0
    while True:
        while not image.size == (640 * 480 * 3):
            continue

        lpos, rpos = process_image(image)

        center = (lpos + rpos) / 2
        angle = -(Width / 2 - center)
        sum_angle += angle
        diff_angle = angle - prev_angle
        c = pid_P * angle + pid_I * sum_angle + pid_D * (diff_angle)
        prev_angle = angle
        # PID Control
        # prev_angle = angle
        # drive(angle, 12)

        if mission == 0:
            if not light_detection_3(image):
                drive(0, 0)  # detect light, not green light
                rospy.sleep(4.5)
                print('----------Light detected and sleep----------------')
                entry_rotary(1, 1.5)
            else:
                drive(c, 6)
        elif mission == 1:
            if detect_stoplineB(image):
                print('--------Stopline detected and sleep-----------')
                drive(0, 0)
                rospy.sleep(4.5)
                flag_ = True
                drive_time = time.time() + 5
                if time.time() > drive_time:
                    flag_ = False
                if flag_:
                    drive(c,6)
                else:
                    cnt += 1
                if cnt == 1:
                    entry_rotary(1, 1)
                    mission_start = time.time()
                elif cnt == 2:
                    entry_rotary(2, 1.5)
            else:
                drive(c, 6)
        elif mission == 2:
            if detect_stoplineB(image):
                print('-----------Change mission 3----------------')
                mission = 3
            else:
                print('-----------In Rotary----------------')
                drive(c, 4)
        elif mission == 3:
            drive(c, 3)
            avoid_obstaclesB()
        elif mission == 4:
            if not light_detection_4(image):
                drive(0, 0)
                rospy.sleep(4.5)
            else:
                drive(c, 3)
        elif mission == 5:
            if limit == 5:
                print('main left')
                #turn_left()
            elif limit == 6:
                print('main right')
                #turn_right()
        elif mission == 6:
            if not scan_front(45, 135, 0.3):
                drive(c, 3)
            else: # wall detect
                print('--------------------------------END')
                drive(0, 0)
                rospy.sleep(100)
        rate.sleep()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()


if __name__ == '__main__':
    start()
