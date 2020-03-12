#include "ros/ros.h"
#include "std_msgs/String.h"
#include <interfaces/Odometry.h>
#include <interfaces/Control.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

using namespace Eigen;

// Define a callback class
class Listener
{
	public:
		void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg);
		void callbackControl(const interfaces::Control::ConstPtr& msg);
		Listener(ros::NodeHandle nh, ros::NodeHandle nhp);
	private:
		// Parameters
		float l_wheel;
		float x_wheel;
		// Old odom pose
		Vector3f pose;
		// Subscriber and Publisher
		ros::Subscriber sub_odom;
		ros::Subscriber sub_control;
		ros::Publisher pub_odom;
		ros::Publisher pub_control;
};
// Constructor
Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Get Parameters
	if (!nhp.getParam("L_wheel", l_wheel)) ROS_ERROR("odometry_transform: Could not find L_wheel parameter!");
	ROS_INFO("odometry_transform: Loaded Parameter\n L_wheel: %f", l_wheel);
	if (!nhp.getParam("X_wheel", x_wheel)) ROS_ERROR("odometry_transform: Could not find X_wheel parameter!");
	ROS_INFO("odometry_transform: Loaded Parameter\n X_wheel: %f", x_wheel);
	// Initialize old pose with zeros
	pose = Vector3f::Zero();
	// Initialize subscribers
	sub_odom = nh.subscribe("gazebo_odom", 10, &Listener::callbackOdom, this);
	sub_control = nh.subscribe("controlData", 10, &Listener::callbackControl, this);
	// Initialize publisher
	pub_odom = nh.advertise<interfaces::Odometry>("odometryData", 10);
	pub_control = nh.advertise<geometry_msgs::Twist>("gazebo_cmd_vel", 10);
}


// Callback functions to get the data from the topics
void Listener::callbackOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	// get message
	nav_msgs::Odometry msg_new = *msg;
	// Allocate data to new pose vector
	Vector3f new_pose = Vector3f::Zero();
	new_pose(0) = msg_new.pose.pose.position.x;
	new_pose(1) = msg_new.pose.pose.position.y;
	Quaternionf quat(msg_new.pose.pose.orientation.w, msg_new.pose.pose.orientation.x, msg_new.pose.pose.orientation.y, msg_new.pose.pose.orientation.z);
	Matrix3f rotMat = quat.toRotationMatrix();
	new_pose(2) = atan2f(rotMat(1,0),rotMat(0,0));
	// Calculate odometry output
	Vector3f diff_pose = new_pose - pose;
	if(diff_pose(2) < -PI) {
		diff_pose(2) = 2*PI + diff_pose(2);
	} else if(diff_pose(2) > PI) {
		diff_pose(2) = diff_pose(2) - 2*PI;
	}
	// TODO: Add for negative ds
	float ds = diff_pose.segment(0,2).norm();
	float dphi = diff_pose(2);
	interfaces::Odometry msg_out;
	msg_out.l_R = ds + l_wheel*dphi;
	msg_out.l_L = ds - l_wheel*dphi;
	// publish data
	pub_odom.publish(msg_out);
	pose = new_pose;
}
void Listener::callbackControl(const interfaces::Control::ConstPtr& msg)
{
	// get message
	interfaces::Control msg_new = *msg;
	// forward message and publish
	geometry_msgs::Twist msg_out;
	msg_out.linear.x = msg_new.v;
	msg_out.angular.z = msg_new.w;
	pub_control.publish(msg_out);
}

int main(int argc, char **argv)
{
	// Initialize the ROS System
	ros::init(argc, argv, "odometryTransfrom");
	// Initialize node handle
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	// Initialize the publisher for the control commands, tell them which message should be published
	// Initialize the subscriber to get the teleop data
	Listener listener(nh, nhp);
	// Let ros spin
	ros::spin();
	return 0;
}
