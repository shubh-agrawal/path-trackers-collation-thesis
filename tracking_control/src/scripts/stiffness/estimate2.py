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
from scipy.optimize import least_squares

wheelbase = 1.983

global steerk
global vyk
global vxk
global rk
global i
global dt
global A
global B

i=-1

data_len = 3000

A = np.zeros(data_len,dtype='float32')
B = np.zeros((data_len,2),dtype='float32')
# parameters
dt = 0.01                        # time tick[s]
L = 1.983                         # Wheel base of the vehicle [m]
max_steer = math.radians(45.0)  # maximum steering angle[rad]

def func(x):
    global A
    global B

    return (A-B.dot(x))

def callback_feedback(data):
    global i
    global steerk
    global vyk
    global vxk
    global rk
    global dt
    global A
    global B

    lf = 1.0
    lr = 1.0
    m = 1400.0
    Iz = 2682

    # state.x = data.pose.pose.position.x
    # state.y = data.pose.pose.position.y
    #print state.x
    # siny = 2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y)
    # cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z) 
    #state.yaw = math.atan2(siny, cosy) 

    # vx =  data.twist.twist.linear.x * math.cos(state.yaw) + data.twist.twist.linear.y * math.sin(state.yaw)
    # vy = data.twist.twist.linear.y * math.cos(state.yaw) - data.twist.twist.linear.x * math.sin(state.yaw)
    # r = data.twist.twist.angular.z
    vx =  data.linear.x
    vy = data.linear.y 
    r = data.angular.y
    steer = data.angular.z
    print "Vx=",vx,"Vy=",vy,"R",r,"Steer:",steer

    if i<data_len/2:
        if i == -1:
            pass

        else:
            A[2*i] = vy - vyk + vxk*rk*dt
            A[2*i+1] = r - rk
            B[2*i][0] = -(lf*rk+vyk)/(m*vxk)+ steerk/m
            B[2*i][1] = (lr*rk - vyk)/(m*vxk)
            B[2*i+1][0] = -(lf*vyk+lf*lf*rk)/(Iz*vxk) + lf*steerk/Iz
            B[2*i+1][1] = (lr*vyk - lr*lr*rk)/(Iz*vxk)
            B[2*i] *= dt
            B[2*i+1] *= dt

        vxk = vx
        vyk = vy
        rk = r
        steerk = steer 
        i+=1

    else:
        C = np.array([1000000,1000000])
        res = least_squares(func,C) 
        print(res.x)
    #
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
    #rospy.Subscriber("base_pose_ground_truth",Odometry, callback_feedback)
    rospy.Subscriber("cmd_delta",Twist, callback_feedback)
    #pub = rospy.Publisher("cmd_delta", Twist, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()