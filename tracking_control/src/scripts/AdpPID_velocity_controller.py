#!/usr/bin/env python
'''
AdpPID_tracker
This code is using adaptive PID for tuning the velocity 
Used along with path tracking controller
Link to reference paper- http://oa.upm.es/30015/1/INVE_MEM_2013_165545.pdf 
Authors - Manthan Patel, Sombit Dey
'''

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import thread
from prius_msgs.msg import Control

# Node name      - controls
# Publish topic  - pid_output (Twist)
# Subscribe topic- base_pose_ground_truth ,cmd_vel (Twist), cmd_delta (Twist)

gear_stat = "F"
global pub
global tar_vel
global tar_delta
act_vel_can = 0
error_sum = 0
prev_error = 0
error_diff = 0
filtered_error = 0
prev_filtered_error = 0
filtered_error_diff = 0
output = 0
wheelbase = 1.958  # in meters
radius = 0
steering_angle = 0
tar_vel = 0  # target velocity
tar_delta = 0  # target delta
kp = 5  # proportionality gain
ki = 0.02  # integral gain
kd = 0.05  # derivative gain
yp = 0.8  # kp gain
yi = 0.02  #Ki gain
yd = 0.5  #Kd gain 
y = 0.8  #low pass filter constant
acc_thershold = 0
brake_threshold = 20
tar_vel = 0


def callback_delta(data):
	"""
	Assigns the value from subscribed topic to the variable tar_delta

	:param tar_delta: (float)
	:param data: (twist)
	"""
	global tar_delta
	tar_delta = data.angular.z


def callback_cmd_vel(data):
	"""
	Assigns the value of velocity from topic cmd_vel to tar_vel

	:param tar_vel: (float)
	:param data: (twist) 
	"""
	global tar_vel
	tar_vel = data.linear.x


def prius_pub(data):
	'''
	publishes the velocity and steering angle
	published on topic : ackermann_cmd_topic
	'''
	global prius_vel
	prius_vel = Control()

	if(data.linear.x > 0):
		prius_vel.throttle = data.linear.x / 100
		prius_vel.brake = 0
		print ("acc")
		print (prius_vel.throttle)

	if(data.linear.x < 0):
		prius_vel.brake = -data.linear.x / 100
		prius_vel.throttle = 0
		print ("brake")
		print (prius_vel.brake)

	prius_vel.steer = data.angular.z / 30
	#print "steering:", prius_vel.steer

	pub.publish(prius_vel)


def callback_feedback(data):
	"""
	Uses the adaptive PID formula and publish the velocity and delta
	on topic pid_output

	:param yaw: (float)
	"""
	global act_vel_can
	global tar_vel
	global wheelbase
	global error_sum
	global error
	global error_diff
	global filtered_error_diff
	global filtered_error
	global prev_filtered_error
	global y
	global output
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
	plot = Twist()
	output = Twist()

	siny = 2.0 * (data.pose.pose.orientation.w *
				  data.pose.pose.orientation.z +
				  data.pose.pose.orientation.x *
				  data.pose.pose.orientation.y)
	cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y *
						data.pose.pose.orientation.y +
						data.pose.pose.orientation.z *
						data.pose.pose.orientation.z)
	yaw = math.atan2(siny, cosy)

	last_recorded_vel = (data.twist.twist.linear.x * math.cos(yaw) +
						data.twist.twist.linear.y * math.sin(yaw))

	act_vel_can = last_recorded_vel

	error = tar_vel - act_vel_can
	error_sum += error
	error_diff = error - prev_error

	filtered_error = error * (1 - y) + prev_error * y  #Low pass filter
	filtered_error_diff = filtered_error - prev_filtered_error
	prev_filtered_error = filtered_error

	if error == 0:

		if tar_vel == 0:
			output.linear.x = 0
		else:
			output.linear.x = output.linear.x - 5

	#Tuning the values of PID constants        
	kp = kp + yp * (error - filtered_error)
	ki = ki + yi * filtered_error
	kd = kd + yd * (error_diff - filtered_error_diff)

	#Limiting the values of kp, ki, kd to a certain range
	if(kp > 25):
		kp = 25

	if(kp < 3):
		kp = 3

	if(ki > 5):
		ki = 10

	if(ki < 0):
		ki = 0

	if(kd > 10):
		kd = 10

	if(kd < 0):
		kd = 0


	if error > 0.01:
		output.linear.x = (kp * error + ki * error_sum + kd * error_diff)
	if error < -0.01:
		output.linear.x = (kp * error + ki * error_sum +
						   kd * error_diff) - brake_threshold

	plot.linear.x = tar_vel  #target velocity
	plot.linear.y = act_vel_can  #current velocity
	plot.linear.z = tar_vel - act_vel_can  # error term
	plot.angular.x = filtered_error

	#Limiting the values of target velocity
	if output.linear.x > 100:
		output.linear.x = 100
	if output.linear.x < -100:
		output.linear.x = -100

	#Limiting the steering angle between [-30,30]    
	output.angular.z = min(30, max(-30, tar_delta)) 

	print output
	prius_pub(output)
	pub1.publish(plot)


def start():
	global pub
	global pub1
	ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/prius')
	rospy.init_node('controls', anonymous=True)
	pub = rospy.Publisher(ackermann_cmd_topic, Control, queue_size=10)
	pub1 = rospy.Publisher('plot', Twist, queue_size=10)
	rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
	rospy.Subscriber("cmd_delta", Twist, callback_delta)
	rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
	rospy.spin()


if __name__ == '__main__':
	start()
