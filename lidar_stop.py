#! /usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan



# Variables
#motor_msg = xycar_motor()
#pub = None
lidar_range = []
flag = 0

def callback(data):
	global lidar_range
	lidar_range = data.ranges

def scan_front(degree_s, degree_e):
	global flag, lidar_range
	flag = 0
	
	for degree in range(degree_s, degree_e):	# scan degree
		if lidar_range[degree] <= 0.4:
			print lidar_range[degree]
			flag = 1
			break
			
	if flag == 1:
		return False
		
	return True
		
#def drive(angle):
#	global motor_msg, pub
#	motor_msg.speed = 3
#	motor_msg.angle = angle
#	pub.publish(motor_msg)
	
#def stop(angle):
#	global motor_msg, pub
#	motor_msg.speed = 0
#	motor_msg.angle = angle
#	pub.publish(motor_msg)

rospy.init_node('lidar_stop', anonymous = True)
rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
#pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)


