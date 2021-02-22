#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg
import numpy as np
from cv_bridge import CvBridge
import cv2
import detect_traffic_light

from sensor_msgs.msg import LaserScan, Image
from xycar_motor.msg import xycar_motor

image = np.empty(shape=[0])
bridge = CvBridge()
distance = []
state = 0
pub = None


# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)

class sensor: 
    def __init__(self):
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback, queue_size = 1)
        
    def img_callback(self, img):
        global image 
        image = bridge.imgmsg_to_cv2(img, "bgr8")

def start():
    global pub
    global image
    angle = 0
    speed = 0

    rospy.init_node('state_node', anonymous=True)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    sensor()
    print ("미니카롱아! 움직여볼까??")
    rospy.sleep(2)
    
    while not rospy.is_shutdown():

        traffic_decision = detect_traffic_light.state_5(image)
        #drive(angle, speed)
        #print (angle,speed)


if __name__ == '__main__':
    start()