#!/usr/bin/env python
'''
sinusoidal_path.py
This code publishes a static sinusoidal path in the form of an array for the bot to follow.
Used for path tracking with controller code
Authors : Adarsh Patnaik
'''
import rospy
import tf
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
import numpy as np
import sys

import math

global tar_vel
tar_vel = float(sys.argv[1])

def frange(x, y, jump):
	'''
	finds the set of points for path generation using fixed jump parameters.
	:params x : initial parameter value
	:params y : final parameter value
	:params jump : constant jump in parameter
	'''
	while x < y:
		yield x
		x += jump


# def set_params(x_offset, y_offset):
# 	'''
# 	Function to set params for path offset optionally

# 	:params x_offset : Offset for x coordinate
# 	:params y_offset : Offset for y coordinate

# 	'''
# 	print 'Do you want to change the path offset'
# 	if raw_input('Press y to change else press n : ') == 'y':
# 		x_offset = raw_input('enter x offset:')
# 		y_offset = raw_input('enter y offset:')

# 	return x_offset, y_offset

def get_quaternion_matrix(data):
	mat = np.array([[(1 - 2*data[1]**2 - 2*data[2]**2), 2*(data[0]*data[1] - data[3]*data[2]), 2*(data[0]*data[2] + data[1] * data[3])], 
					[2*(data[0] * data[1] + data[2] * data[3]), (1 - 2*data[0]**2 - 2*data[2]**2), 2*(data[1] * data[2] - data[0] * data[3])], 
					[2*(data[0] * data[2] - data[1]*data[3]), 2*(data[1] * data[2] + data[0] * data[3]), (1 - 2*data[1]**2 - 2*data[0]**2)]])

	return mat

def main():
	'''
	gets path coordinates and publishes them in form of an array.

	'''
	global x_offset 
	global y_offset 
	rospy.init_node('astroid_curve_publisher')
	
	path_pub = rospy.Publisher('astroid_path', Path, queue_size=5)
	vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
	odom_msg = rospy.wait_for_message('/base_pose_ground_truth', Odometry)
	x_offset = odom_msg.pose.pose.position.x
	y_offset = odom_msg.pose.pose.position.y
	path = Path()
	quat = quaternion_from_euler(0, 0, -0.785398)

	path.header.frame_id = rospy.get_param('~output_frame', 'map')
	radius = rospy.get_param('~radius', 50.0) # radius of path
	resolution = rospy.get_param('~resolution', 0.01) # constant jump value for parameter
	holonomic = rospy.get_param('~holonomic', False)
	offset_x = rospy.get_param('~offset_x', x_offset) # get x offset from params
	offset_y = rospy.get_param('~offset_y', y_offset) # get y offset from params
	update_rate = rospy.get_param('~update_rate', 100) # rate of path publishing
	has_initialize = True
	# loop to get the path coordinates
	for t in frange(0, math.pi * 2, resolution):
		x = 25 * t 
		y = 25 * math.sin(t) 
		if has_initialize:
			old_x = x
			old_y = y
			has_initialize = False

		init_vector = np.array([x, y, 0])
		init_vector.resize(3, 1)
		rot_vector = np.dot(get_quaternion_matrix(quat), init_vector) 

		pose = PoseStamped()
		pose.pose.position.x = rot_vector[0] + float(offset_x)
		pose.pose.position.y = rot_vector[1] + float(offset_y)

		
		
		yaw = 0.0
		if holonomic:
			yaw = -math.sin(t) / math.cos(t)
		else:
			if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
				yaw = math.atan2(old_y - y, old_x - x)
			else:
				yaw = math.atan2(y - old_y, x - old_x)
		
		q = tf.transformations.quaternion_from_euler(0, 0, yaw)
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]
		path.poses.append(pose)

		old_x = x
		old_y = y

	vel = Twist()
	vel.linear.x = tar_vel
	
	r = rospy.Rate(update_rate)
	while not rospy.is_shutdown():
		path.header.stamp = rospy.get_rostime()
		path_pub.publish(path)
		vel_pub.publish(vel)
		
		r.sleep()
	
if __name__ == '__main__':
	main()

