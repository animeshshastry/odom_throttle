#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

#include <sstream>

double accx, accy, accz;
double current_time, prev_time, dt;
double qw, qx, qy, qz;
double RTe3_1, RTe3_2, RTe3_3;

// nav_msgs::Odometry odom_msg;
geometry_msgs::Point VIO_pos;
geometry_msgs::Vector3 VIO_vel;
geometry_msgs::Vector3 VIO_Euler;

void acc_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	accx = msg->linear_acceleration.z;
	accy = msg->linear_acceleration.x;
	accz = msg->linear_acceleration.y;

	accx -= RTe3_1*(9.81);
	accy -= RTe3_2*(9.81);
	accz -= RTe3_3*(9.81);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// odom_msg = &msg;
	VIO_pos = msg->pose.pose.position;
	VIO_vel = msg->twist.twist.linear;

	qw = msg->pose.pose.orientation.w;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;

	RTe3_1 = 2.0*(qx*qz - qy*qw);
	RTe3_2 = 2.0*(qy*qz + qw*qx);
	RTe3_3 = 1.0 - 2.0*(qx*qx + qy*qy);

	VIO_Euler.x = atan2(2 * (qw * qx + qy * qz), 1.0f - 2 * (qx * qx + qy * qy));
	VIO_Euler.y = asin(2.0f * (qw * qy - qz * qx));
	VIO_Euler.z = atan2(2 * (qx * qy + qw * qz), 1.0f - 2 * (qy * qy + qz * qz));
}

void propagate(){

	current_time = ros::Time::now().toSec();
	dt = current_time - prev_time;
	prev_time = current_time;

	//VIO_pos.x += VIO_vel.x*dt;
	//VIO_pos.y += VIO_vel.y*dt;
	//VIO_pos.z += VIO_vel.z*dt;

	VIO_vel.x += accx*dt;
	VIO_vel.y += accy*dt;
	VIO_vel.z += accz*dt;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "VIO_odom");
	ros::NodeHandle n;
	// ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("VIO_odom", 1000);
	ros::Publisher pos_pub = n.advertise<geometry_msgs::Point>("VIO_pos", 1);
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3>("VIO_vel", 1);
	ros::Publisher Euler_pub = n.advertise<geometry_msgs::Vector3>("VIO_Euler", 1);
	ros::Subscriber odom_sub = n.subscribe("/camera/odom/sample", 1, odom_callback);
	ros::Subscriber acc_sub = n.subscribe("/camera/accel/sample", 1, acc_callback);

	ros::Rate loop_rate(100);
	// ros::AsyncSpinner spinner(2); // Use 2 threads

	while (ros::ok())
	{

		// ROS_INFO("%s", msg.data.c_str());

		// odom_pub.publish(odom_msg);

		ros::spinOnce();
		propagate();
		pos_pub.publish(VIO_pos);
		vel_pub.publish(VIO_vel);
		Euler_pub.publish(VIO_Euler);
		// // spinner.start();

		// ros::Duration(0.002).sleep(); // sleep
		loop_rate.sleep();
	}


	return 0;
}