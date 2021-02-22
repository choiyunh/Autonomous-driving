#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg
import numpy as np
from cv_bridge import CvBridge
import cv2

#우리가 만든 미션수행 노드들
import detect_stop
from mission_gigi import minicar_mission
from detect_line_gigi import line_dic_soo
from detect_traffic_light_gigi import detect_trff
from lidar import lidar

from sensor_msgs.msg import LaserScan, Image
from xycar_motor.msg import xycar_motor

image = np.empty(shape=[0])
bridge = CvBridge()
scan = []
state = 0
pub = None

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

class Sensor: 
    def __init__(self):
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback, queue_size = 1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
     
    def img_callback(self, img):
        global image
        image = bridge.imgmsg_to_cv2(img, "bgr8")

    def scan_callback(self, distance):
        global scan
        scan = distance.ranges

def start():
    global pub
    global image
    global state
    global scan
    angle = 0
    speed = 0
    done = 0

    rospy.init_node('state_node', anonymous=True)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    Sensor()
    Mission = minicar_mission()
    Line = line_dic_soo()
    Lidar = lidar()
    Trff = detect_trff()

    # state = int(input('0 is line_dic, 1 ~ 5 is mission1 ~ mission5, 6 ~ 7 is trff, 9 is lidar : '))
    state = 9

    print "------자------드가자------!"
    rospy.sleep(1)
    
    while not rospy.is_shutdown():
        if state == 8:
            input('----------mission is done!----------')

        #1번 신호등
        elif state == 1:
            print('mission1')
            while not rospy.is_shutdown():
                angle, speed, done = Mission.mission1(image)
                print(angle, speed)
                if done == 1:
                    done = 0
                    state = 8
                    break

        #2번 신호등
        elif state == 2:
            print('mission2')
            while not rospy.is_shutdown():
                angle, speed, done = Mission.mission2(image, scan)
                print(angle, speed)
                if done == 1:
                    done = 0
                    state = 8
                    break

        #회전 교차로
        elif state == 3:
            print('mission3')
            while not rospy.is_shutdown():
                angle, speed, done = Mission.mission3(image, scan)
                print(angle, speed)
                if done == 1:
                    done = 0
                    state = 8
                    break

        #차선 변경
        elif state == 4:
            print('mission4')
            while not rospy.is_shutdown():
                angle, speed, done = Mission.mission4(image, scan)
                print(angle, speed)
                if done == 1:
                    done = 0
                    state = 8
                    break

        #좌우 신호등 
        elif state == 5:
            print('mission5')
            while not rospy.is_shutdown():
                angle, speed, done = Mission.mission5(image, scan)
                print(angle, speed)
                if done == 1:
                    done = 0
                    state = 8
                    break

        #3구 신호등
        elif state == 6:
            print('trff1')
            while not rospy.is_shutdown():
                st = Trff.state_1(image)
                print (st)
                if done == 1:
                    done = 0
                    state = 8
                    break

        #4구 신호등
        elif state == 7:
            print('trff2')
            while not rospy.is_shutdown():
                st = Trff.state_5(image)
                print (st)
                if done == 1:
                    done = 0
                    state = 8
                    break
        #차선인식
        elif state == 0:
            print('line_dic')
            while not rospy.is_shutdown():
                angle, speed = Line.dict_line(image)
                drive(angle, speed)
                # print(angle, speed)
                # rospy.sleep(0.1)
                if done == 1:
                    done = 0
                    state = 8
                    break

        #라이다
        elif state == 9:
            print('lidar')
            while not rospy.is_shutdown():
                dis = Lidar.det_lidar(scan,-5,5,0.5,3)
                print(dis)
                if done == 1:
                    done = 0
                    state = 8
                    break

if __name__ == '__main__':
    start()
