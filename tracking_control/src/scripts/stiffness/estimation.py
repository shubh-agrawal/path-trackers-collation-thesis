#!/usr/bin/env python

'''
Node Name   - path_tracking_lqr
Publishers  - 0
Subscribers -
Authors : Aditya Rathore, Het Shah   
'''

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int64
import rospy, math
import numpy as np
import scipy.linalg as la

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

wheelbase = 1.983

global steer
global ep
global cp
global x_p
global last_recorded_vel
global state
global e
global e_th
global pe
global pe_th
global i
global tar_vel
global angle

i=0
pe = 0
pe_th = 0
state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)
last_recorded_vel = 0
x_p = Path()
flag = 1

# LQR parameter
Q = np.eye(4)
R = np.eye(1)

# parameters
dt = 0.01                        # time tick[s]
L = 1.983                         # Wheel base of the vehicle [m]
max_steer = math.radians(45.0)  # maximum steering angle[rad]

def callback_cmd_vel(data):
    """
    Assigns the value of velocity from topic cmd_vel to tar_vel

    :param tar_vel: (float)
    :param data: (twist) 
    """
    global tar_vel
    tar_vel = data.linear.x
    global angle
    angle = data.angular.z
    angle = min(30, max(-30, angle))

def callback_feedback(data):

    global i
    global tar_vel
    global angle
    
    i+=1
    state.x = data.pose.pose.position.x
    state.y = data.pose.pose.position.y
    #print state.x
    siny = 2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y)
    cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z) 
    state.yaw = math.atan2(siny, cosy) 

    state.v = data.twist.twist.linear.x * math.cos(state.yaw) + data.twist.twist.linear.y * math.sin(state.yaw)
    vy = data.twist.twist.linear.y * math.cos(state.yaw) - data.twist.twist.linear.x * math.sin(state.yaw)
    print "Vx=",state.v,"Vy=",vy

    # if (i/100) % 2 == 0:
    #     angle = +30
    # elif (i/100) % 2 == 1:
    #     angle = -30

    cmd_steer = Twist()
    cmd_steer.linear.x=state.v
    cmd_steer.linear.y=vy
    cmd_steer.linear.z=tar_vel
    cmd_steer.angular.y=data.twist.twist.angular.z
    cmd_steer.angular.z = angle
    pub.publish(cmd_steer)
    
    # if len(x_p.poses) > 0: 

    #     ind, e = calc_nearest_index(state, x_p)
    #     cross2 = [(state.x - x_p.poses[ind].pose.position.x),
    #           (state.y - x_p.poses[ind].pose.position.y)]
    #     cross = [math.cos(state.yaw), math.sin(state.yaw)]
    #     cross_prod = cross[0] * cross2[1] - cross[1] * cross2[0]
    #     if (cross_prod > 0):
    #         e = -e
    #     steer_path = math.atan2(x_p.poses[ind].pose.position.y - x_p.poses[ind-1].pose.position.y, x_p.poses[ind].pose.position.x - x_p.poses[ind-1].pose.position.x )
    #     e_th = (state.yaw - steer_path)*(-1)
    #     delta_lqr  = lqr_steering_control(state, e, e_th, pe, pe_th, x_p)

    #     cmd_steer = Twist()
    #     cmd_steer.angular.z = delta_lqr*180.0/math.pi 
    #     #print "Delta:",cmd_steer.angular.z
    #     print "Error:", e
    #     print "Error theta", e_th
    #     pub.publish(cmd_steer)
    #     pe = e
    #     pe_th = e_th

def main():
    global pub

    print("LQR steering control tracking start!!")
    rospy.init_node('path_tracking_lqr',anonymous = True)
    rospy.Subscriber("base_pose_ground_truth",Odometry, callback_feedback)
    rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
    pub = rospy.Publisher("cmd_delta", Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()