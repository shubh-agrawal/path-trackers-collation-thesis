#!/usr/bin/env python
'''
PID_Control
This code applies PID controller on velocity.
Authors : Het Shah
'''
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import thread

# Node name       - controls
# Publishe topic  - pid_output (Twist)
# Subscribe topic - base_pose_ground_truth , cmd_vel, cmd_delta (Twist)

gear_stat = "F"
tar_vel = 0 # target velocity
tar_omega = 0 # target omega
act_vel_can = 0 # measured velocity 
error_sum = 0 
prev_error = 0
error_diff = 0
output = 0 
wheelbase = 1.958  # in meters
radius = 0 # radius of curvature
steering_angle = 0
kp = 1000.0 # proportional gain
ki = 1.5 # integral gain
kd = 0.8 # differential gain
acc_thershold = 0 # threshold velocity for acceleration
brake_threshold = 0 # threshold veocity for braking
global pub
global tar_vel
global tar_omega
tar_vel = 0
tar_omega = 0

def convert(v, omega, wheelbase):
	'''
	convert omega to angle in degrees
	: params v [float]
	: params omega [float]
	: params wheelbase [float]
	returns angle in degrees
	'''
    
    if omega == 0 or v == 0:
    	return 0
    
    radius = v / omega
    # checking the sign of radius of the path
    if omega > 0 and v > 0:
        radius = -abs(radius)
    if omega > 0 and v < 0:
        radius = abs(radius)
    if omega < 0 and v > 0:
        radius = abs(radius)
    if omega < 0 and v < 0:
        radius = -abs(radius)

    return math.atan(wheelbase / radius) * 180 / 3.14

def callback_feedback(data):
	'''
	applies PID control to the velocity
	:params data [Twist]
	:params output [Twist]
	:params plot [Twist]  
	'''
    global act_vel_can # velcoity of the bot
    global tar_vel # target velocity
    global tar_delta # target delta
    global tar_omega # target omega
    global wheelbase # wheel base for ackerman bot
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
    global acc_thershold # threshold for acceleration
    global brake_threshold # threshold for braking
    global act_velocity

    # conversion from quarternion to euler
    siny = +2.0 * (data.pose.pose.orientation.w *
                   data.pose.pose.orientation.z +
                   data.pose.pose.orientation.x *
                   data.pose.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
                         data.pose.pose.orientation.y +
                         data.pose.pose.orientation.z *
                         data.pose.pose.orientation.z)
    yaw = math.atan2(siny, cosy)

    last_recorded_vel = (data.twist.twist.linear.x * math.cos(yaw) +
                         data.twist.twist.linear.y * math.sin(yaw))
    # last_recorded_ang = data.twist.twist.angular.z

    act_vel_can = last_recorded_vel
  
    plot = Twist()
    output = Twist()
    # PID control applied
    error = tar_vel - act_vel_can
    error_sum += error
    error_diff = error - prev_error
    prev_error = error
    if error == 0:
        print ("e")
        if tar_vel == 0:
            output.linear.x = 0
        else:
            output.linear.x = output.linear.x - 5
    # PID maths
    if error > 0.01:
        print ("e1")
        output.linear.x = (kp * error + ki * error_sum + kd * error_diff)
    if error < -0.01:
        print ("e2")
        output.linear.x = ((kp * error + ki * error_sum + kd * error_diff) -
                           brake_threshold)

    plot.linear.x = tar_vel
    plot.linear.y = act_vel_can
    plot.linear.z = tar_vel - act_vel_can  # error term

    # output.linear.x = 100
    print output.linear.x
    # output.linear.x = int(output.linear.x)
    # thresholding the linear velocity
    if output.linear.x > 100:
        output.linear.x = 100
    if output.linear.x < -100:
        output.linear.x = -100
    # thresholding the angular steering
    output.angular.z = min(
        80, max(-80, convert(tar_vel, tar_omega, wheelbase)))

    pub.publish(output)
    pub1.publish(plot)

def callback_cmd_vel(data):
	'''
	Sets the target velocity and steering angle
	:params data [Twist]
	:params tar_vel [float]
	:params tar_omega [float]
	'''
    global tar_vel
    global tar_omega
    tar_vel = data.linear.x
    tar_omega = data.angular.z

def start():
    global pub
    global pub1
    rospy.init_node('controls', anonymous=True)
    pub = rospy.Publisher('pid_output', Twist, queue_size=10)
    pub1 = rospy.Publisher('plot', Twist, queue_size=10)
    rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
    rospy.spin()


if __name__ == '__main__':
        # thread.start_new_thread( tar_update , ())
    start()