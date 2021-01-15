#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>

// nav_msgs::Odometry odom_msg;
geometry_msgs::Point VIO_pos;
geometry_msgs::Vector3 VIO_vel;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // odom_msg = &msg;
  VIO_pos = msg->pose.pose.position;
  VIO_vel = msg->twist.twist.linear;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "VIO_odom");
  ros::NodeHandle n;
  // ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("VIO_odom", 1000);
  ros::Publisher pos_pub = n.advertise<geometry_msgs::Point>("VIO_pos", 0);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3>("VIO_vel", 0);
  ros::Subscriber sub = n.subscribe("/camera/odom/sample", 0, odom_callback);

  ros::Rate loop_rate(100);
  // ros::AsyncSpinner spinner(2); // Use 2 threads

  while (ros::ok())
  {

    // ROS_INFO("%s", msg.data.c_str());

    // odom_pub.publish(odom_msg);

    ros::spinOnce();
    pos_pub.publish(VIO_pos);
    vel_pub.publish(VIO_vel);
	// // spinner.start();

    loop_rate.sleep();
  }


  return 0;
}