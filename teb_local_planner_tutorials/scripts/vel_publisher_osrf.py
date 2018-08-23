#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from geometry_msgs.msg import Twist
# from ackermann_msgs.msg import AckermannDriveStamped
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega

  if omega > 0 and v > 0 :
    radius = -abs(radius)
  if omega > 0 and v < 0 :
    radius = abs(radius)
  if omega < 0 and v > 0 :
    radius = abs(radius)
  if omega < 0 and v < 0 :
    radius  = -abs(radius)  
         
  return math.atan(wheelbase / radius) 

def odom_callback(data):
  global correct_v
  global correct_yaw
  global alpha_v
  global alpha_yaw
  global v
  global yaw
  global last_recorded_vel

  last_recorded_vel =data.twist.twist.linear.x
  correct_yaw = alpha_yaw*(yaw - data.twist.twist.angular.z)


def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  global correct_v
  global correct_yaw
  global v
  global yaw
  global last_recorded_vel
  global last_recorded_ang
  global kp
  global ki
  global kd
  global dt
  global integral
  global last_error
  prius_vel = Control()
  target_vel = data.linear.x
  print ("Command Velocity: "),
  if(abs(target_vel) < 0.2):
    target_vel = 0
    print ("0.0")
  if(target_vel >= 0.2):
    prius_vel.shift_gears = 2
    print("+"),
    print (target_vel)
  if(target_vel <= -0.2):
    prius_vel.shift_gears = 3
    print (target_vel)
    



  integral = integral + ki*(target_vel - last_recorded_vel)
  integral = min(3, max(-3, integral))
  v = kp*(target_vel - last_recorded_vel) + integral + kd*(target_vel - last_recorded_vel - last_error)
  v = min(1, max(-1, v))
  last_error = (target_vel - last_recorded_vel)
  #v = data.linear.x + correct_v
  
  yaw = data.angular.z + correct_yaw
  steering = convert_trans_rot_vel_to_steering_angle(v, yaw, wheelbase)
  if(target_vel <= -0.2):
    v = v * -1.0
  # msg = AckermannDriveStamped()
  # msg.header.stamp = rospy.Time.now()
  # msg.header.frame_id = frame_id
  # msg.drive.steering_angle = steering
  # msg.drive.speed = v
  
  
  prius_vel.throttle = v;
  prius_vel.brake = 0;
  prius_vel.steer = steering/3.14;
  if(prius_vel.throttle<0):
    prius_vel.brake=1
  #pub.publish(msg)
  pub.publish(prius_vel)




if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_ackermann_drive')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/prius')
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    frame_id = rospy.get_param('~frame_id', 'odom')

    alpha_v = 0.1
    alpha_yaw = 0.1

    correct_v = 0
    correct_yaw = 0
    v=0
    yaw=0   
    last_recorded_vel = 0
    last_recorded_ang = 0
    integral = 0;
    kp = 5 #change these
    ki = 4
    kd = 0.2
    last_error = 0.0
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)

    
    rospy.Subscriber("/base_pose_ground_truth", Odometry, odom_callback,queue_size=1)    
    
    #pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    pub = rospy.Publisher(ackermann_cmd_topic, Control, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass