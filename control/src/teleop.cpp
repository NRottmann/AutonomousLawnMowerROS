#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <interfaces/Control.h>
#include <interfaces/Odometry.h>
#include <interfaces/Bumper.h>

// Define a callback class
class TeleopListener
{
	public:
		void callback(const geometry_msgs::Twist::ConstPtr& twist);
		void calculateMotorCommands(interfaces::Control &msg);
		void calculateMotorCommandsRamp(interfaces::Control &msg);
	private:
	    geometry_msgs::Twist tele_twist;
		float v0 = 0.0f;
		float w0 = 0.0f;
};
// Define Functions
// Callback function to get the data from the topic
void TeleopListener::callback(const geometry_msgs::Twist::ConstPtr& twist)
{
	tele_twist.linear = twist->linear;
	tele_twist.angular = twist->angular;
}
// Calculates the desired speed
void TeleopListener::calculateMotorCommands(interfaces::Control &msg) 
{
	float vMax = 0.5;
	float wMax = 0.5;
	msg.v = tele_twist.linear.x * vMax;;
	msg.w = tele_twist.angular.z * wMax;;
}
// Define a callback class
class OdometryListener
{
public:
	void callback(const interfaces::Odometry::ConstPtr& msg_in);
	void actualizePosition();
	void getParam(ros::NodeHandle nhp);
private:
	interfaces::Odometry msg_store;
	float x = 0.0f;
	float y = 0.0f;
	float phi = 0.0f;
	// Parameter
	float L = 0.0f;
};
void OdometryListener::callback(const interfaces::Odometry::ConstPtr& msg_in)
{
	msg_store = *msg_in;
	// ROS_INFO("l_R: %f,  l_L: %f", msg_in->l_R, msg_in->l_L);
}
// Actualize position
void OdometryListener::actualizePosition() 
{
	float ds = 0.5 * (msg_store.l_R - msg_store.l_L);
	float dphi = (msg_store.l_R + msg_store.l_L) / (2*L);

	x = x + sin(phi) * ds;
	y = y + cos(phi) * ds;
	phi = phi + dphi;

	// ROS_INFO("Position: [%3.2f] [%3.2f] [%3.2f]", x, y, phi);
}
void OdometryListener::getParam(ros::NodeHandle nhp)
{
	if (!nhp.getParam("robot/L", L)) ROS_ERROR("Could not find robot/L parameter!");
	ROS_INFO("Teleop: Loaded Parameter\n L: %f", L);
}


int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "teleop");
  // Initialize node handle
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Initialize the publisher for the control commands, tell them which message should be published
  ros::Publisher tele_pub = nh.advertise<interfaces::Control>("controlData", 1000);
  // Initialize the subscriber to get the teleop data
  TeleopListener listener;
  ros::Subscriber sub = nh.subscribe("/joy_teleop/cmd_vel", 1000, &TeleopListener::callback, &listener);
  // Initialize the subscriber to get the odometry messages
  OdometryListener listenerOdo;
  listenerOdo.getParam(nhp);
  ros::Subscriber sub_odo = nh.subscribe("odometryData", 1000, &OdometryListener::callback, &listenerOdo);
  // Loop rate
  double frequency;
  if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("Could not find system/frequency parameter!");
  ros::Rate loop_rate(frequency);
  // Start the loop
  while (ros::ok())
  {
	// Initialize increment message
	interfaces::Control msg;
	// Calculate Increments
	listener.calculateMotorCommands(msg);
    // publish message
	tele_pub.publish(msg);
	// Subscribe using the callback functions
    ros::spinOnce();
	// Actualize position information
	listenerOdo.actualizePosition();
	// Sleep until rate is reached
    loop_rate.sleep();
  }
  return 0;
}
