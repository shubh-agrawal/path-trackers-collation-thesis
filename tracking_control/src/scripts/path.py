#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math

def frange(x, y, jump):
    while x < y:
        yield x
        x += jump

def main():
    rospy.init_node('astroid_curve_publisher')
    
    path_pub = rospy.Publisher('astroid_path', Path, queue_size=10)
    path = Path()

    path.header.frame_id = rospy.get_param('~output_frame', 'map')
    radius = rospy.get_param('~radius', 50.0)
    resolution = rospy.get_param('~resolution', 0.01)
    holonomic = rospy.get_param('~holonomic', False)
    offset_x = rospy.get_param('~offset_x', 155.0)
    offset_y = rospy.get_param('~offset_y', 280.0)
    update_rate = rospy.get_param('~update_rate', 2.5)

    has_initialize = True
    
    for t in frange(0, math.pi * 2, resolution):
        # x = radius * math.cos(t) ** 3 + offset_x
        # y = radius * math.sin(t) ** 3 + offset_y
        x = 7 * t + offset_x
        y = 10 * math.sin(t) + offset_y
        if has_initialize:
            old_x = x
            old_y = y
            has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    
    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        path.header.stamp = rospy.get_rostime()
        path_pub.publish(path)
        
        print 'published'
        # for i in range(len(path.poses)):
        #     print [path.poses[i].pose.position.x, path.poses[i].pose.position.y]
        # print 'end'
        r.sleep()
    
if __name__ == '__main__':
    main()

