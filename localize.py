import rospy
import cv2
import numpy as np
import time
import sys
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
 

def callback_scan(msg):
#    for depth in msg.ranges:
	global pos_x, pos_y
	x = (msg.ranges[90]*400)
	if(x >= 0 and x <= 10000):
		pos_x = int(x)
	y = (msg.ranges[180]*400)
	if(y >= 0 and y <= 10000):
		pos_y = int(y)
	# print(pos_x, pos_y)

def callback_distance_sensor(msg):
	global pos_z
	pos_z = msg.range
	# z = (msg.range*400)
	# if(z >= 0 and z <= 10000):
	# 	pos_z = int(z)
	# print(pos_z)

def callback_slam_pose(msg):
	global pub_pose, rot_x, rot_y
	msg.pose.position.z = pos_z
	msg.pose.orientation.x = rot_x
	msg.pose.orientation.y = rot_y
	pub_pose.publish(msg)
	# print(msg)

def callback_imu_pose(msg):
	global pub_pose, rot_x, rot_y
	rot_x = msg.pose.orientation.x
	rot_y = msg.pose.orientation.y
	# print(rot_x)

def visualize_2d(x, y):
	img = np.zeros((800,800,3), np.uint8)
	cv2.circle(img,(x, y), 10, (255,255,255), -1)
	cv2.namedWindow("Draw", cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Draw', 600,600)
	cv2.imshow('Draw', img)


	if cv2.waitKey(1) & 0xFF == ord('q'):
		exit()

def visualize_3d(x, y, z):
	img = np.zeros((800,800,3), np.uint8)
	cv2.circle(img,(x, y), 10, (255,255,255), -1)
	fig = plt.figure()
	ax = plt.axes(projection='3d')
	zline = [z]
	xline = [x]
	yline = [y]
	ax.plot3D(xline, yline, zline, 'gray')
	plt.show()
	# cv2.namedWindow("Draw", cv2.WINDOW_NORMAL)
	# cv2.resizeWindow('Draw', 600,600)
	# cv2.imshow('Draw', img)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		exit()
 
def node():
	global pos_x, pos_y, pos_z, rot_x, rot_y, pub_pose
	pos_x, pos_y, pos_z, rot_x, rot_y = 0,0,0,0,0
	rospy.init_node('localization')
	pub_pose = rospy.Publisher("/mavros/vision_pose/debug", PoseStamped, queue_size=1)
	sub_scan = rospy.Subscriber('/scan', LaserScan, callback_scan)
	sub_imu_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_imu_pose)
	sub_slam_pose = rospy.Subscriber('/robot_pose', PoseStamped, callback_slam_pose)
	sub_dist_sens = rospy.Subscriber('/mavros/distance_sensor/benewake_tfmini', 
			Range, 
			callback_distance_sensor)

	while not rospy.is_shutdown():
		if(len(sys.argv) > 1):
			if(sys.argv[1] == "2d"):
				visualize_2d(pos_x, pos_y)
			if(sys.argv[1] == "3d"):
				visualize_3d(pos_x, pos_y, pos_z)
		# rospy.loginfo(str(pos_x))
		rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass