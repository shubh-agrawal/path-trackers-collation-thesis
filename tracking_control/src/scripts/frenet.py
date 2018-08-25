import frenet_optimal_trajectory as planner

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import cubic_spline_planner
import rospy
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

import tf

global odom,path_pub,global_path_pub
odom = Odometry()
path_pub = None
global_path_pub = None

def odom_callback(odommsg):
	# global odom 
	odom = odommsg

def calc_dis(p1x,p1y,p2x,p2y):
    dx = math.fabs(p1x - p2x)
    dy = math.fabs(p1y - p2y)
    dist = math.sqrt(dx**2 + dy**2)
    return dist

def find_nearest_in_global_path(global_x,global_y):
    '''
    Finds the nearest point in the global path form the bot
    global_x - array of points in global path
    global_y - array of points in global path

    returns the point at the minimum distance and the minimum distance
    '''
    # global odom 

    bot_x = odom.pose.pose.position.x
    bot_y = odom.pose.pose.position.y
    min_x = None
    min_y = None
    min_id = None

    min_dis = float('inf')
    i = 0

    for gx,gy in zip(global_x,global_y):
        dis = calc_dis(gx,gy,bot_x,bot_y)
        if dis < min_dis:
            min_dis = dis
            min_x = gx 
            min_y = gy
            min_id = i 
        i+=1

    assert min_x is not None and min_y is not None

    return (min_x,min_y,min_dis,min_id)

def calc_s(ptx,pty,global_x,global_y):
    '''
    Finds the corresponding s for each point in the global path array
    '''

    s = 0.0
    if global_x[0] == ptx and global_y[0] == pty:
        return s

    for i in range(1,len(global_x)):
        gx = global_x[i]
        gy = global_y[i]

        s+=calc_dis(gx,gy,global_x[i-1],global_y[i-1])

        if gx == ptx and gy == pty:
            break
    return s

def get_bot_yaw():
    # global odom 
    pose = odom.pose.pose
    quaternion = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    return yaw

def global_path_tangent(global_x,global_y):
    theta_g = []
    for i in range(len(global_x) - 1):
        dx = global_x[i+1] - global_x[i]
        dy = global_y[i+1] - global_y[i]

        theta = math.atan2(dy,dx)
        theta_g.append(theta)

    if len(theta_g) > 0:
        theta_g.append(theta_g[-1])

    return theta_g

def find_params(global_x,global_y):
    '''
    global_x - array of points in global path
    global_y - array of points in global path

    returns s,d,s_dot,d_dot,d_dot_dot
    s - from function
    d - minimum distance along global path
    s_dot - v_bot*cos(delta_theta)
    d_dot - v_bot*sin(delta_theta)
    d_dot_dot - 0 for the time being
    '''
    min_x,min_y,d,min_id = find_nearest_in_global_path(global_x,global_y)
 
    s = calc_s(min_x,min_y,global_x,global_y)

    v_bot = odom.twist.twist.linear.x 
    bot_yaw = get_bot_yaw()
    theta = global_path_tangent(global_x,global_y)

    global_tangent_angle = theta[min_id]

    s_dot = v_bot*1.0*math.cos(bot_yaw - global_tangent_angle)
    d_dot = v_bot*1.0*math.sin(bot_yaw - global_tangent_angle)

    d_dot_dot = 0.0

    return (s,d,s_dot,d_dot,d_dot_dot)

def oc_callback(msg):
    '''
    Updates the obstacles array with the costmap data
    ''' 
    # global ob
    occupied_thresh = 50 #Change according to customisation 

    ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    # ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
    ogrid_mpc = np.array(msg.info.resolution,dtype=np.float64) #meter per cell

    ob = np.argwhere(ogrid >= occupied_thresh)
    ob = ob.astype(float)

    if(ob is not None):
        if(ob.shape[0] > 0 and ob.shape[1] > 0):
            ob*=ogrid_mpc
        else:
            ob = np.array([[]])
    else:
        ob = np.array([[]])

    # print('andar')
    # print(ob)
    # print(ob.shape)

def main():
	### Frenet Variables ###
	 # way points
    wx = [0.0, 10.0, 20.5, 35.0, 70.5]
    wy = [0.0, -6.0, 5.0, 6.5, 0.0]

    # obstacle lists
    ### Updated with the global variable obs ###
    ### This has been deprecated ###
    # ob = np.array([[20000,10000]])   

    tx, ty, tyaw, tc, csp = planner.generate_target_course(wx, wy)

    # initial state
    c_speed = 10.0 / 3.6  # current speed [m/s]
    c_d = 0.0 # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current latral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 20.0  # animation area length [m]
    #########################

    ### ROS Interface ###
    # global path_pub,odom,ob
    global ob
    ob = None
    
    ### Initialize Node ###
    rospy.init_node('frenet_planner') 

    ### Publisher ###
    path_pub=rospy.Publisher('/frenet_path', Path, queue_size=10) 
    global_path_pub=rospy.Publisher('/global_path', Path, queue_size=10) 

    ### Subscriber ###
    rate = rospy.Rate(10)
    rospy.Subscriber("base_pose_ground_truth",Odometry,odom_callback)
    # rospy.Subscriber("move_base/local_costmap/costmap",OccupancyGrid,oc_callback)

    while not rospy.is_shutdown():
        s0, c_d, c_speed, c_d_d, c_d_dd = find_params(tx, ty)

        # print('bahar')
        # print(ob)
        debug_vals = [s0,c_d,c_speed,c_d_d,c_d_dd]
        print(tx)
        # print(debug_vals)

    	### Get The Path From The Frenet Optimal Trajectory ###
    	path = planner.frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)

        if path is None:
            print('path is none')
            continue

    	### Publishing Path Details ###
    	path_msg = Path()
        global_path_msg = Path()
    	path_msg.header.frame_id = 'odom'
        global_path_msg.header.frame_id = 'odom'
    	current_odom_position = PoseStamped() #Stores The Car's Current Position
    	current_odom_position.pose.position.x = odom.pose.pose.position.x
    	current_odom_position.pose.position.y = odom.pose.pose.position.y
    	path_msg.poses.append(current_odom_position)

    	### Append The Full Frenet Path ###
    	for x,y in zip(path.x,path.y):
            loc = PoseStamped()
            loc.pose.position.x = x + odom.pose.pose.position.x
            loc.pose.position.y = y + odom.pose.pose.position.y
            path_msg.poses.append(loc)

        for x,y in zip(tx,ty):
            loc = PoseStamped()
            loc.pose.position.x = x + odom.pose.pose.position.x
            loc.pose.position.y = y + odom.pose.pose.position.y
            global_path_msg.poses.append(loc)

    	path_pub.publish(path_msg)
        global_path_pub.publish(global_path_msg)
    	###############################

    	### TODO : Make These Updates ###
    	# s0 = path.s[1]
     #    c_d = path.d[1]
     #    c_d_d = path.d_d[1]
     #    c_d_dd = path.d_dd[1]
     #    c_speed = path.s_d[1]

        
        ### Spin ###
        # rospy.spin()

        ###TODO : Publish The Data To /cmd_vel

        ######################################
        
    	# rate.sleep()

if __name__ == '__main__':
    main()
