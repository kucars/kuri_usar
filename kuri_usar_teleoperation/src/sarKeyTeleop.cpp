#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <geometry_msgs/TwistStamped.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include "sarKeyTeleop.h"

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

sarKeyTeleop::sarKeyTeleop():
    ph_("~")
{
    ROS_INFO_STREAM ("Initialize Teleoperation UAV");
    std::cout<< "Read Parameters" << std::endl;

    ph_.param("scale_angular" , a_scale_, 1.0);
    ph_.param("scale_linear"  , l_scale_, 1.0);

    std::cout<< "scale_angular = " << a_scale_ << std::endl;
    std::cout<< "scale_linear  = " << l_scale_ << std::endl;
    velocty_pub = ph_.advertise<geometry_msgs::TwistStamped>("/uav_1/mavros/setpoint_velocity/cmd_vel", 1000);
    pose_sub_ = ph_.subscribe("/uav_1/mavros/local_position/pose" ,1, &sarKeyTeleop::poseCallback, this );


}
void sarKeyTeleop::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion q( msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    std::cout << "pitch" << pitch << std::endl ; 
    std::cout << "roll" << roll << std::endl ; 
    std::cout << "yaw" << yaw << std::endl ; 
    std::cout << "yaw in degrees: " << yaw * 180 / 3.14  << std::endl ; 
    
}
void sarKeyTeleop::watchdog()
{
    boost::mutex::scoped_lock lock(publish_mutex_);
    if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) &&
            (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    {
        geometry_msgs::TwistStamped vel_msg;
        sendVel(vel_msg);
    }
}

void sarKeyTeleop::sendVel(geometry_msgs::TwistStamped & vel)
{
    vel.twist.linear.x  = l_scale_ * vel.twist.linear.x;
    vel.twist.linear.y  = l_scale_ * vel.twist.linear.y;
    vel.twist.linear.z  = l_scale_ * vel.twist.linear.z;
    vel.twist.angular.z = a_scale_ * vel.twist.angular.z;
    velocty_pub.publish(vel);
   // interface_.velCom(vel);
    return;
}

void sarKeyTeleop::keyLoop()
{
    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the UAV.");

    while (ros::ok())
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        ROS_DEBUG("value: 0x%02X\n", c);
        geometry_msgs::TwistStamped vel_msg;
        switch(c)
        {
        case Arrow_U    : linear_x = 1.0 ; linear_y = 0.0; linear_z = 0.0; angular_z=0.0 ; break;
        case Arrow_D    : linear_x = -1.0 ; linear_y = 0.0; linear_z = 0.0; angular_z=0.0 ; break;
        case Arrow_L    : linear_x = 0.0 ; linear_y = 1.0; linear_z = 0.0; angular_z=0.0 ; break;
        case Arrow_R    : linear_x = 0.0 ; linear_y = -1.0; linear_z = 0.0; angular_z=0.0 ; break;	
        case KEY_t      : linear_x = 0.0 ; linear_y = 0.0; linear_z = 1.0; angular_z=0.0 ; break;
        case KEY_l      : linear_x = 0.0 ; linear_y = 0.0; linear_z = -1.0; angular_z=0.0 ; break;
       	case KEY_u      : linear_x = 0.0 ; linear_y = 0.0; linear_z = 0.0; angular_z=3.14 ; break;
        case KEY_y      : linear_x = 0.0 ; linear_y = 0.0; linear_z = 0.0; angular_z=-3.14 ; break;
       }
        vel_msg.twist.linear.x = linear_x*cos(yaw ) - linear_y * sin(yaw) ; 
        vel_msg.twist.linear.y = linear_x*sin (yaw) + linear_y * cos(yaw) ;
	vel_msg.twist.linear.z = linear_z ;
        vel_msg.twist.angular.z = angular_z;

        boost::mutex::scoped_lock lock(publish_mutex_);
        if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) {
            first_publish_ = ros::Time::now();
        }
        last_publish_ = ros::Time::now();
        sendVel(vel_msg);
    }

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sar_teleop");


    sarKeyTeleop teleop_uav;
    ros::NodeHandle n;
    signal(SIGINT,quit);

    boost::thread my_thread(boost::bind(&sarKeyTeleop::keyLoop, &teleop_uav));
    ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&sarKeyTeleop::watchdog, &teleop_uav));

    ros::spin();

    my_thread.interrupt() ;
    my_thread.join() ;

    return(0);
}
