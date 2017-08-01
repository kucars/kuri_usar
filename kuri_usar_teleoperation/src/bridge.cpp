#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

float lin_vel_x , lin_vel_y , lin_vel_z, ang_vel_x , ang_vel_y, ang_vel_z ; 

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
   lin_vel_x = msg->linear.x  ; 
   lin_vel_y = msg->linear.y  ; 
   lin_vel_z = msg->linear.z  ; 
   ang_vel_x = msg->angular.x ; 
   ang_vel_y = msg->angular.y ; 
   ang_vel_z = msg->angular.z ; 
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "velocityBridge");   
  ros::NodeHandle n;   
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::TwistStamped>("/uav_1/mavros/setpoint_velocity/cmd_vel", 1000);
  ros::Rate loop_rate(10);   

  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::TwistStamped msg;
    msg.twist.linear.x = lin_vel_x ; 
    msg.twist.linear.y = lin_vel_y ; 
    msg.twist.linear.z = lin_vel_z ; 
    msg.twist.angular.x = ang_vel_x ; 
    msg.twist.angular.y = ang_vel_y ; 
    msg.twist.angular.z = ang_vel_z ; 
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
