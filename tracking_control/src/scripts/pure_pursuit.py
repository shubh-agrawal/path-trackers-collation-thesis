#!/usr/bin/env python
'''
This is the implementation of pure pursuit controller for path tracking
Link to reference paper: https://www.ri.cmu.edu/pub_files/2009/2
/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf

Authors: Adarsh Patnaik, Anand Jhunjhunwala

'''

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from prius_msgs.msg import Control

# node name: path_tracking
# Publish Topic: cmd_delta
# Subscribe Topic: base_pose_ground_truth, astroid_path

max_vel = 6.0 # maximum linear velocity
global steer
k = 0.8 # constant for relating look ahead distance and velocity
wheelbase = 1.983 # wheel base for the vehicle
d_lookahead = 0.08 # look ahead distance to calculate target point on path
print ("start")


def callback_feedback(data):
    '''
    Assigns the position of the robot to global variables from odometry.
    :param x_bot [float]
    :param y_bot [float]
    :param yaw [float]
    :param vel [float]
    '''
    global x_bot
    global y_bot
    global yaw
    global vel

    x_bot = data.pose.pose.position.x
    y_bot = data.pose.pose.position.y
    # quarternion to euler conversion
    siny = +2.0 * (data.pose.pose.orientation.w *
                   data.pose.pose.orientation.z +
                   data.pose.pose.orientation.x *
                   data.pose.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
                         data.pose.pose.orientation.y +
                         data.pose.pose.orientation.z *
                         data.pose.pose.orientation.z)
    yaw = math.atan2(siny, cosy) # yaw in radians
    # vel = data.twist.twist.linear.x * math.cos(yaw)
    # + data.twist.twist.linear.y * math.sin(yaw)
    # printing the odometry readings
    print 'x of car:', x_bot
    print 'y of car:', y_bot
    print 'angle of car:', yaw
    print 'vel of car:', data.twist.twist.linear.x,
    data.twist.twist.linear.y,
    data.twist.twist.linear.z
    print 'c'

def dist(a, x, y):
    '''
    Calculates distance between two points.
    :param a [float]
    :param x [float]
    :param y [float]
    '''
    # calculate distance
    return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5


def callback_path(data):
    '''
    calculates target path point and the steering angle
    :param data [Path]
    :param ep [float]
    :param cp [int]
    '''
    global ep # min distance
    global cp # index of closest point

    x_p = data
    # calculate minimum distance 
    distances = []
    for i in range(len(x_p.poses)):
        a = x_p.poses[i]
        distances += [dist(a, x_bot, y_bot)]
    ep = min(distances)
    cp = distances.index(ep)
    print 'old index:', cp
    # calculate index of target point on path
    cmd = Twist()
    cmd1 = Twist()
    prius_vel = Control()
    L = 0
    Lf = k * max_vel + d_lookahead

    while Lf > L and (cp + 1) < len(x_p.poses):
        dx = data.poses[cp + 1].pose.position.x - \
            data.poses[cp].pose.position.x
        dy = data.poses[cp + 1].pose.position.y - \
            data.poses[cp].pose.position.y
        L += math.sqrt(dx ** 2 + dy ** 2)
        cp = cp + 1
    print len(x_p.poses)
    print 'new index is:', cp

    goal_point = [x_p.poses[cp].pose.position.x,
                  x_p.poses[cp].pose.position.y]
    print 'current goal is:', goal_point
    error = [goal_point[0] - x_bot, goal_point[1] - y_bot]
    print error
    steer_angle = pure_pursuit(goal_point)

    print "steer_angle :", steer_angle * 180 / math.pi
    cmd.angular.z = min(30, max(-30, steer_angle * 180 / math.pi))
    cmd.linear.y = math.sqrt(error[0]**2 + error[1]**2)
    print 'omega:', cmd.angular.z
    pub1.publish(cmd)
    
    print "cmd published"

    # print (ep)
    print x_p.poses[cp].pose.orientation


def pure_pursuit(goal_point):
    '''
    Calculates the steering angle required for path tracking
    :params goal_point [float,float] goal point coordinates
    :params Delta [float] steering angle in radians 
    '''
    tx = goal_point[0]
    ty = goal_point[1]
    print 'yaw:', yaw
    # measuring the slope of path point
    print 'slope:', math.atan2(ty - y_bot, tx - x_bot)
    # measuring heading angle 
    alpha = math.atan2(ty - y_bot, tx - x_bot) - yaw
    print 'alpha:', alpha
    Lf = k * max_vel + d_lookahead
    # measuring the steering angle using pure pursuit controller
    Delta = math.atan2(2.0 * wheelbase * math.sin(alpha) / Lf, 1)
    print 'Delta:', Delta
    return Delta


def start():
    global pub1
    
    rospy.init_node('path_tracking', anonymous=True)
    
    pub1 = rospy.Publisher('cmd_delta', Twist, queue_size=10)
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
    rospy.Subscriber("astroid_path", Path, callback_path)

    rospy.spin()


if __name__ == '__main__':
    start()
