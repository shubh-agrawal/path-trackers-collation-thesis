'''
Geometric method for path tracking with Stanley steering control
Link to paper- https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
Authors - Manthan Patel, Sombit Dey
'''

#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int64

# Node name      - path_tracking
# Publish topic  - cmd_delta (Twist)
# Subscribe topic- base_pose_ground_truth , astroid_path

global steer
kp = 2  #gain parameter
alpha = 0.5
wheelbase = 1.983  #in meters

def callback_feedback(data):
    """
    calculates the current bot orientation(bot_theta) and current bot
    velocity(bot_vel) by converting from quaternion coordinates

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param data: (twist)
    :param bot_theta: (float) current orientation of bot
    :param bot_vel: (float) current velocity of bot

    """
    global x
    global y
    global bot_theta
    global bot_vel
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    siny = 2.0 * (data.pose.pose.orientation.w *
                  data.pose.pose.orientation.z +
                  data.pose.pose.orientation.x *
                  data.pose.pose.orientation.y)
    cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y *
                        data.pose.pose.orientation.y +
                        data.pose.pose.orientation.z *
                        data.pose.pose.orientation.z)
    bot_theta = math.atan2(siny, cosy)
    bot_vel = data.twist.twist.linear.x * math.cos(bot_theta) + data.twist.twist.linear.y * math.sin(bot_theta)


def dist(a, x, y):
    """
    calculates the euclidian distance between 2 points

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param a: () contains the coordinates of other point
    """
    return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5


def callback_path(data):
    """
    publishes the steering angle for the bot after calculating it
    :param ep: (float) distance of the closest point
    :param cp: (int) index of the closest point in the list
    :param distances: (list-float) 
    :param cmd: (twist)  varaible which will be published
    :param data: (twist)

    """
    global ep
    global cp
    global bot_theta1


    x_p = data

    distances = []
    for i in range(len(x_p.poses)):
        a = x_p.poses[i]
        distances += [dist(a, x, y)]
    ep = min(distances)

    cp = distances.index(ep)
    
    cmd = Twist()
    cross2 = [(x - data.poses[cp].pose.position.x),
              (y - data.poses[cp].pose.position.y)]
    cross = [math.cos(bot_theta), math.sin(bot_theta)]
    cross_prod = cross[0] * cross2[1] - cross[1] * cross2[0]
    if (cross_prod > 0):
        ep = -ep
    #print ("cp %f ep %f" % (cp, ep))

    siny = +2.0 * (x_p.poses[cp].pose.orientation.w *
                   x_p.poses[cp].pose.orientation.z +
                   x_p.poses[cp].pose.orientation.x *
                   x_p.poses[cp].pose.orientation.y)
    cosy = +1.0 - 2.0 * (x_p.poses[cp].pose.orientation.y *
                         x_p.poses[cp].pose.orientation.y +
                         x_p.poses[cp].pose.orientation.z *
                         x_p.poses[cp].pose.orientation.z)

    #steer_path = math.atan2(siny, cosy)
    
    steer_path = math.atan2(x_p.poses[i].pose.position.y - x_p.poses[i-1].pose.position.y, x_p.poses[i].pose.position.x - x_p.poses[i-1].pose.position.x )

    bot_theta1 = (bot_theta + math.pi) % math.pi  #converts bot_theta [-pi to pi] to [0 to pi]
    steer_err = (bot_theta1 - steer_path) * (-1)

    tan = math.atan(ep / kp)   
     
    delta = (alpha*steer_err + tan)
    #print ("steer err %f bot_theta %f steer_path %f" % (steer_err, bot_theta1, steer_path))
    
    delta = delta * 180 / 3.14  #converting delta into degrees from radian
    print delta
    cmd.angular.z = delta
    pub1.publish(cmd)


def start():
    global pub1
    global pub2
    rospy.init_node('path_tracking', anonymous=True)
    pub1 = rospy.Publisher('cmd_delta', Twist, queue_size=10)
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
    rospy.Subscriber("astroid_path", Path, callback_path)
    rospy.spin()


if __name__ == '__main__':
    start()
