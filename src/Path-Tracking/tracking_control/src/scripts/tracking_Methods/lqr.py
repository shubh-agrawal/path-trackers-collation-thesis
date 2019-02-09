#!/usr/bin/env python

'''
Node Name   - path_tracking_lqr
Publishers  - cmd_delta
Subscribers - base_pose_ground_truth, path
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
#import control

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

pe = 0
pe_th = 0
state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)
x_p = Path()

'''
LQR parameters
Q matrix penalises the error input state x
R matrix penalises the  
'''
Q = 100*np.eye(4)
Q[1, 1] = 0
Q[2, 2] = 0
Q[3, 3] = 0
R = np.matrix([[1]])

# Other parameters
dt = 0.01                         # time tick[s]
L = 1.983                         # Wheel base of the vehicle [m]
max_steer = 30.0                  # maximum steering angle[degrees]
global pub,pub2
n=0
ep_sum=0
ep_max=0

def path_length_distance(a,b):

    return (((a.pose.position.x - b.pose.position.x)**2) + ((a.pose.position.y - b.pose.position.y)**2))**0.5

def calc_path_length(data):
    global path_length
    path_length = []

    for i in range(len(data.poses)):
        if i == 0:
            path_length.append(0)

        else:
            path_length.append(path_length[i-1] + path_length_distance(data.poses[i], data.poses[i-1]))

def callback_feedback(data):
    '''
    calculates steering angle
    :param data [Odometry]
    :param x_p [path]
    :param e [cross track error]
    :param pe [ previous cross track error]
    :param e_th [angle error]
    :param pe_th [ previous angle error]
    '''
    global x_p
    global state
    global e_th
    global e
    global pe_th
    global pe
    global pub
    global path_length
    global n
    global ep_max
    global ep_sum
    
    state.x = data.pose.pose.position.x
    state.y = data.pose.pose.position.y

    # quarternion to euler conversion
    siny = +2.0 * (data.pose.pose.orientation.w *
                   data.pose.pose.orientation.z +
                   data.pose.pose.orientation.x *
                   data.pose.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
                         data.pose.pose.orientation.y +
                         data.pose.pose.orientation.z *
                         data.pose.pose.orientation.z)
    state.yaw = math.atan2(siny, cosy) # yaw in radians  
    
    state.v = (data.twist.twist.linear.x * math.cos(state.yaw) +
                data.twist.twist.linear.y * math.sin(state.yaw))

    if len(x_p.poses) > 0:
        ind, e = calc_nearest_index(state, x_p)

        #Figure out wheter the cross track error is +ve or -ve
        cross2 = [(state.x - x_p.poses[ind].pose.position.x),
              (state.y - x_p.poses[ind].pose.position.y)]
        cross = [math.cos(state.yaw), math.sin(state.yaw)]
        cross_prod = cross[0] * cross2[1] - cross[1] * cross2[0]
        if (cross_prod > 0):
            e = -e

        path_angle = math.atan2(x_p.poses[ind].pose.position.y - x_p.poses[ind-1].pose.position.y,
                                x_p.poses[ind].pose.position.x - x_p.poses[ind-1].pose.position.x )

        e_th = pi_2_pi(state.yaw - path_angle)

        
        delta_lqr  = lqr_steering_control(state, e, e_th, pe, pe_th, x_p) 
        cmd_steer = Twist()
        cmd_steer.angular.z = delta_lqr*180.0/math.pi 
        pub.publish(cmd_steer)
        print ("Steering angle(degrees) %f " % (cmd_steer.angular.z))
        pe = e
        pe_th = e_th

        calc_path_length(x_p)
        global ep_max
        # distances = []
        # for i in range(len(x_p.poses)):
        #     a = x_p.poses[i]
        #     distances += [dist(a, state.x, state.y)]
        # ep = min(distances)
        if (e > ep_max):
            ep_max = e

        n = n + 1
        ep_sum = ep_sum + e
        ep_avg = ep_sum / n

        # cp = distances.index(ep)
        # cross2 = [(state.x - x_p.poses[cp].pose.position.x),
        #           (state.y - x_p.poses[cp].pose.position.y)]
        # cross = [math.cos(state.yaw), math.sin(state.yaw)]
        # cross_prod = cross[0] * cross2[1] - cross[1] * cross2[0]
        # if (cross_prod > 0):
        #     ep = -ep

        # steer_path = math.atan2(x_p.poses[cp].pose.position.y - x_p.poses[cp-1].pose.position.y, x_p.poses[cp].pose.position.x - x_p.poses[cp-1].pose.position.x )
        # steer_err = (state.yaw - steer_path) * (-1)
        # tan = math.atan(ep / kp)   
        # delta = (steer_err + tan)
        # delta = delta * 180 / 3.14  #converting delta into degrees from radian
        # delta = min(40,max(-40,delta))


        cross_err = Twist()
        cross_err.linear.x = e
        cross_err.angular.x = ep_max
        cross_err.angular.y = ep_avg
        cross_err.linear.y = e_th
        cross_err.linear.z = path_length[ind]

    

def callback_path(data):
    global x_p
    x_p = data

def pi_2_pi(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi

def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE) by iterative method
    """
    P = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Pn = A.T * P * A - A.T * P * B * la.inv(R + B.T * P * B) * B.T * P * A + Q
        if (abs(Pn - P)).max() < eps:
            P = Pn
            break
        P = Pn
    return Pn

def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = np.matrix(la.inv(B.T * X * B + R) * (B.T * X * A))  
    eigVals, eigVecs = la.eig(A - B * K)

    return K, X, eigVals

def dist(a,x,y):
    return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5

def lqr_steering_control(state, e, e_th, pe, pe_th, x_p):
    '''
    calculates the steering angle
    Input : state, previous and current cross track error,
            previous and current angle error 
    '''
    v = state.v

    A = np.matrix(np.zeros((4,4)))
    B = np.matrix(np.zeros((4, 1)))

    '''
    Simple Kinematic Model
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    
    B[3, 0] = v / L  
    '''

    # Dynamic Model converted to state space assuming constant velocity 1m/s, Cf = Cr = 155494, m = 1140 kg, Iz = 1436
    A[0,0] = 1.0
    A[0,1] = 0.0037
    A[0,2] = 0.0963
    A[0,3] = 0.0003
    A[1,2] = 1.0
    A[1,3] = 0.0034
    A[2,2] = 1.0
    A[2,3] = 0.0963

    B[0,0] = 0.050025784700646
    B[1,0] = 0.539884777742374 
    B[2,0] = 0.041458048416231
    B[3,0] = 0.429184549356149

    K, _, _ = dlqr(A, B, Q, R)

    #State matrix x
    x = np.matrix(np.zeros((4, 1)))

    x[0, 0] = -e
    x[1, 0] = -(e - pe) / dt
    x[2, 0] = e_th
    x[3, 0] = (e_th - pe_th) / dt

    #Linear Output
    u = -K * x   

    #calc steering input    
    fb =u[0, 0]
    
    # Feedforward Term ff 
    ff = 0
    #ff = math.atan2(L * k, 1)

    delta = ff + fb

    return delta

def calc_nearest_index(state, x_p):
    '''
    calculates minimum distance between path and a given point
    '''
    distances = []

    for i in range(len(x_p.poses)):
        a = x_p.poses[i]
        distances += [dist(a, state.x, state.y)]   
    ep = min(distances) 
    cp = distances.index(ep)
    return cp , ep

def main():
    global pub,pub2

    print("LQR steering control tracking start!!")
    rospy.init_node('path_tracking_lqr',anonymous = True)
    rospy.Subscriber("base_pose_ground_truth",Odometry, callback_feedback, queue_size=10)
    rospy.Subscriber("astroid_path",Path, callback_path, queue_size=10)
    pub = rospy.Publisher("cmd_delta", Twist, queue_size=10)
    pub2 = rospy.Publisher('cross_track_error', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()