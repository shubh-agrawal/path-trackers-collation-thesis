#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sstream>

float left_vel;
float right_vel;
//float avg_vel;

//std_msgs::Float32 left_vel;
//std_msgs::Float32 right_vel;

void chatterCallback1(const std_msgs::Float32::ConstPtr& msg)
{
  left_vel = msg->data;
}

void chatterCallback2(const std_msgs::Float32::ConstPtr& msg)
{
  right_vel = msg->data;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "uniform_vel");
  
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("l_vel", 1000, chatterCallback1);
  ros::Subscriber sub2 = n.subscribe("r_vel"  , 1000, chatterCallback2);

  std_msgs::Float32 avg_vel;
  ros::Publisher avg_vel_pub = n.advertise<std_msgs::Float32>("odom/avg_vel", 1000);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {    

    avg_vel.data = (left_vel + right_vel)/2;
  	//avg_vel.data = 0;
    avg_vel_pub.publish(avg_vel);

    ros::spinOnce();
    //ros::spin();

    loop_rate.sleep();
    
  }

  return 0;
}

