#!/usr/bin/env python
'''
PID_MIT_tracker
This code is using adaptive PID with MIT rule for tuning the velocity 
Used along with path tracking controller 
Authors - Adarsh Patnaik,     
'''
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import thread

# Node name       - controls
# Published topic  - pid_output (Twist)
# Subscriber topic - cmd_vel, cmd_delta, base_pose_ground_truth 

gear_stat = "F"
tar_vel = 0 # target velocity
tar_omega = 0 # target omega
act_vel_can = 0 # current velocity
error_sum = 0
prev_error = 0
error_diff = 0
output = 0
wheelbase = 1.958  # in meters
radius = 0 # radius of curvature of path
steering_angle = 0 # steering angle

kp = 8.0 # proprtional gain
ki = 2.0 # integral gain
kd = 0.2 # derivative gain

yp = 20.0 # kp gain
yi = 0.5 # ki gain
yd = 0.1 # kd gain

acc_thershold = 0 #threshold for acceleration
brake_threshold = 20 # threshold for braking
global pub

tar_vel = 0
tar_omega = 0


def callback_feedback(data):
	'''
	Applies adaptive PID to velcity input from odom readings and publishes.
	:params data [Odometry]
	:params output [Twist]
	:params plot [Twist]
	'''
	global act_vel_can
	global tar_vel
	global tar_omega
	global wheelbase
	global error_sum
	global error
	global error_diff
	global output
	global i
	global flag
	global kp
	global ki
	global kd
	global pub
	global prev_error
	global gear_stat
	global acc_thershold
	global brake_threshold
	global act_velocity
	global yp
	global yi
	global yd
	
	siny = 2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z 
				+ data.pose.pose.orientation.x * data.pose.pose.orientation.y)
	cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y *
		data.pose.pose.orientation.y +
		data.pose.pose.orientation.z *
		data.pose.pose.orientation.z)
	yaw = math.atan2(siny, cosy)

	last_recorded_vel = (data.twist.twist.linear.x * math.cos(yaw) +
		data.twist.twist.linear.y * math.sin(yaw))

	act_vel_can = last_recorded_vel

	plot = Twist()
	output = Twist()

	error = tar_vel - act_vel_can
	error_sum += error
	error_diff = error - prev_error
	prev_error = error
	if error == 0:
		if tar_vel == 0:
			output.linear.x = 0
		else:
			output.linear.x = output.linear.x - 5
	# updating kp, ki, kd using MIT rule
	kp = kp + yp * error * error
	ki = ki + yi * error * error_sum
	kd = kd + yd * error * error_diff
	print kp
	print ki
	print kd
	# PID on velocity with updated parameters
	if error > 0.01:
		output.linear.x = (kp * error + ki * error_sum + kd * error_diff)
	if error < -0.01:
		output.linear.x = ((kp * error + ki * error_sum + kd * error_diff) -
			brake_threshold)

	plot.linear.x = tar_vel
	plot.linear.y = act_vel_can
	plot.linear.z = tar_vel - act_vel_can  # error term
	# thresholding the forward velocity
	if output.linear.x > 100:
		output.linear.x = 100
	if output.linear.x < -100:
		output.linear.x = -100
	# thresholding the angle
	output.angular.z = min(30.0, max(-30.0, tar_delta))

	pub.publish(output)
	pub1.publish(plot)


def callback_cmd_vel(data):
	"""
	Assigns the value of velocity from topic cmd_vel to tar_vel

	:param tar_vel: (float)
	:param data: (twist) 
	"""
	global tar_vel
	tar_vel = data.linear.x 



def callback_delta(data):
	"""
	Assigns the value from subscribed topic to the variable tar_delta

	:param tar_delta: (float)
	:param data: (twist)
	"""
	global tar_delta
	tar_delta = data.angular.z


def start():
	global pub
	global pub1
	rospy.init_node('controls', anonymous=True)
	pub = rospy.Publisher('pid_output', Twist, queue_size=10)
	pub1 = rospy.Publisher('plot', Twist, queue_size=10)
	rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
	rospy.Subscriber("cmd_delta", Twist, callback_delta)
	rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)

	rospy.spin()


if __name__ == '__main__':
	start()
