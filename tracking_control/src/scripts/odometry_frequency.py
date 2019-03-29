import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path


def callback_feedback(data):

	pub.publish(data)
	r.sleep()


def start():
	global pub
	global r
	rospy.init_node('odometry_freq', anonymous=True)
	r = rospy.Rate(10)
	pub = rospy.Publisher('base_pose_ground_truth1', Odometry, queue_size=5)
	rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
	rospy.spin()


if __name__ == '__main__':
	start()