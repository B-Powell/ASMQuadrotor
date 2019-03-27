import rospy
import cv2
import numpy as np
import time

from sensor_msgs.msg import LaserScan
 

def callback(msg):
#    for depth in msg.ranges:
	global pos_x, pos_y
	x = (msg.ranges[90]*400)
	if(x >= 0 and x <= 10000):
		pos_x = int(x)
	y = (msg.ranges[180]*400)
	if(y >= 0 and y <= 10000):
		pos_y = int(y)
	print(pos_x, pos_y)
#	visualize(pos_x, pos_y)

def visualize(x, y):
	img = np.zeros((800,800,3), np.uint8)
	cv2.circle(img,(x, y), 10, (255,255,255), -1)
	cv2.namedWindow("Draw", cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Draw', 600,600)
	cv2.imshow('Draw', img)

	# time.sleep(0.0001)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		exit()
 
global pos_x, pos_y
pos_x, pos_y = 0,0
rospy.init_node('localization')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
