#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
import thread
import time

# Node name       - controls
# Publishe topic  - pid_output (Twist)
# Subscribe topic - velocity_can (Twist), cmd_vel1 (Twist) [vel in kmph, omega in rad/sec]
#!/usr/bin/env python
global pub,pub1
count=0
def state():
    global count   
    count = count + 1
    if count > 1000:
        count =0
    if count > 500:
        return 1
    else :
        return -1
  


def callback_feedback(data):
    output = Twist()
    output.linear.x = state()
    output.angular.z = 0
    pub.publish(output)
    print "Published"
    info_c=Point32()
    # conversion of odometry readings from quarternion to euler
    siny = +2.0 * (data.pose.pose.orientation.w *
                   data.pose.pose.orientation.z +
                   data.pose.pose.orientation.x *
                   data.pose.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
                         data.pose.pose.orientation.y +
                         data.pose.pose.orientation.z *
                         data.pose.pose.orientation.z)
    yaw = math.atan2(siny, cosy)

    vel = (data.twist.twist.linear.x * math.cos(yaw) +
                         data.twist.twist.linear.y * math.sin(yaw))
    info_c.x = output.linear.x
    info_c.y= vel
    pub1.publish(info_c) 


def start():
    global pub,pub1
    rospy.init_node('controls',anonymous = True)
    pub = rospy.Publisher('pid_output', Twist, queue_size=10)
    pub1 = rospy.Publisher('info', Point32, queue_size=10)
    rospy.Subscriber("/Odometry/filtered1", Odometry, callback_feedback)
    rospy.spin()
if __name__ == '__main__':
	start()
