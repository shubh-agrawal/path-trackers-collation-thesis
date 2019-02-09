#!/usr/bin/env python
'''
path.py
This code publishes a static parabolic path(sharp turn) in the form of an array for the bot to follow.
Used for path tracking with controller code
Authors : Adarsh Patnaik
'''
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math

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


def set_params(x_offset, y_offset):
	'''
	Function to set params for path offset optionally

	:params x_offset : Offset for x coordinate
	:params y_offset : Offset for y coordinate

	'''
	print 'Do you want to change the path offset'
	if raw_input('Press y to change else press n : ') == 'y':
		x_offset = raw_input('enter x offset:')
		y_offset = raw_input('enter y offset:')

	return x_offset, y_offset


def main():
	'''
	gets path coordinates and publishes them in form of an array.

	'''
	x_offset = 155
	y_offset = 280
	x_offset, y_offset = set_params(x_offset,y_offset)
	rospy.init_node('astroid_curve_publisher')
	
	path_pub = rospy.Publisher('astroid_path', Path, queue_size=10)
	path = Path()

	path.header.frame_id = rospy.get_param('~output_frame', 'map')
	radius = rospy.get_param('~radius', 50.0) # radius of path
	resolution = rospy.get_param('~resolution', 0.01) # constant jump value for parameter
	holonomic = rospy.get_param('~holonomic', False)
	offset_x = rospy.get_param('~offset_x', x_offset) # get x offset from params
	offset_y = rospy.get_param('~offset_y', y_offset) # get y offset from params
	update_rate = rospy.get_param('~update_rate', 100) # rate of path publishing
	has_initialize = True
	# loop to get the path coordinates
	for t in frange(0, 17, resolution):
		x = offset_x + t
		y = offset_y + (17 * t - 0.8*t**2) * 0.1
		if has_initialize:
			old_x = x
			old_y = y
			has_initialize = False

		pose = PoseStamped()
		pose.pose.position.x = x
		pose.pose.position.y = y
		
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
	
	r = rospy.Rate(update_rate)
	while not rospy.is_shutdown():
		path.header.stamp = rospy.get_rostime()
		path_pub.publish(path)
		
		r.sleep()
	
if __name__ == '__main__':
	main()

