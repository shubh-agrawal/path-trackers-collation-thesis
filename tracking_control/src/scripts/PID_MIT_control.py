#!/usr/bin/env python
'''
This code implements the MIT rule in adaptive PID control.
Applies adaptive PID with MIT rule on velocity.
Reference paper:
Author : Het Shah
'''
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import thread

# Node name       - controls
# Published topic  - pid_output, plot
# Subscribed topic - base_pose_ground_truth, cmd_vel 

gear_stat = "F"
tar_vel = 0 # target velocity
tar_omega = 0 # target omega
act_vel_can = 0
error_sum = 0
prev_error = 0
error_diff = 0
output = 0
wheelbase = 1.958  # in meters
radius = 0 #radius of path
steering_angle = 0 # steering angle

kp = 8.0 # proportional gain
ki = 2.0 # integral gain
kd = 0.2 # derivative gain

yp = 20.0 # kp gain
yi = 0.5 # ki gain
yd = 0.1 # kd gain

acc_thershold = 0 # threshold for acceleration
brake_threshold = 20 # threshold for braking
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
    # checking sign of the radius of path
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
	applies adaptive PID control wiht MIT rule to the velocity
	:params data [Twist]
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
    # convert quarternion to euler
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

    act_vel_can = last_recorded_vel
  
    plot = Twist()
    output = Twist()
    # apply PID to velocity
    error = tar_vel - act_vel_can
    error_sum += error
    error_diff = error - prev_error
    prev_error = error
    if error == 0:
        if tar_vel == 0:
            output.linear.x = 0
        else:
            output.linear.x = output.linear.x - 5
    # Updating kp, ki, kd using MIT rule
    kp = kp + yp * error * error
    ki = ki + yi * error * error_sum
    kd = kd + yd * error * error_diff
    print kp
    print ki
    print kd

    if error > 0.01:
        output.linear.x = (kp * error + ki * error_sum + kd * error_diff)
    if error < -0.01:
        output.linear.x = ((kp * error + ki * error_sum + kd * error_diff) -
                           brake_threshold)

    plot.linear.x = tar_vel
    plot.linear.y = act_vel_can
    plot.linear.z = tar_vel - act_vel_can  # error term
    # thresholding velocity between -100 and 100
    if output.linear.x > 100:
        output.linear.x = 100
    if output.linear.x < -100:
        output.linear.x = -100
    # thresholding angle 
    output.angular.z = min(
        80, max(-80, convert(tar_vel, tar_omega, wheelbase)))

    pub.publish(output)
    pub1.publish(plot)

def callback_cmd_vel(data):
	"""
    Assigns the value of velocity and omega from topic cmd_vel to tar_vel 
    and tar_omega

    :param tar_vel: (float) target velocity
    :param tar_omega: (float) target omega
    :param data: (twist) 
    """
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
    start()
