#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from geometry_msgs.msg import Twist
# from ackermann_msgs.msg import AckermannDriveStamped


# def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
#   if omega == 0 or v == 0:
#     return 0

#   radius = v / omega
#   return math.atan(wheelbase / radius)


def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  
  # v = data.linear.x
  # steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  msg = Twist()
  # msg.Header.stamp = data.header
  # msg.header.frame_id = frame_id
  # msg.drive.steering_angle = steering
  # msg.drive.speed = v
  msg.linear.x=data.linear.x*18.0/5.0
  msg.angular.z=data.angular.z;
  
  pub.publish(msg)
  




if __name__ == '__main__': 
  try:
    
    rospy.init_node('mps2kmph')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    # ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    # wheelbase = rospy.get_param('~wheelbase', 1.0)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher("/cmd_vel1",Twist, queue_size=1)
    
    # rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

