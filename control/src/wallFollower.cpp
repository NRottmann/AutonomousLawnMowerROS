#include "ros/ros.h"
#include <interfaces/Control.h>
#include <interfaces/Odometry.h>
#include <interfaces/Sensor.h>
#include <interfaces/Bumper.h>
#include <math.h>

#define PI 3.14159265

// This wallFollower uses the left sensor for wall following, thus it drives along the border clockwise

// Define a callback class
class Listener
{
	public:
		void callbackSensor(const interfaces::Sensor::ConstPtr& msg_in);		// callback function to receive sensor measurements
		void callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in);	// callback function to receive odometry data from the motor
		void callbackBumper(const interfaces::Bumper::ConstPtr& msg_in);		// callback function to receive bumper information (generated using the IMU)
		interfaces::Control getMotorCom();										// function to calculate motor commands
		Listener(ros::NodeHandle nhp);											// this is the constructor
	private:
		interfaces::Sensor msg_sensor;
		interfaces::Odometry msg_odometry;
		interfaces::Bumper msg_bumper;
		float mean = 0.0f;						// mean of measurements of the right sensors
		float d = 0.0f;							// direction of rotation
		float vCurrent = 0.0f;					// current velocity
		float v0;								// Speed limit
		float w0;								// Angular velocity limit
		int sensorTreshold;						// sensor treshold
		int count = 0;							// counter
		bool wallFollowing = false;				// Assume we do not start at the wall
};
// Constructor
Listener::Listener(ros::NodeHandle nhp) {
	if (!nhp.getParam("control/v0", v0)) ROS_ERROR("wallFollower: Could not find control/v0 parameter!");
	ROS_INFO("wallFollower: Loaded Parameter\n v0: %f", v0);
	if (!nhp.getParam("control/w0", w0)) ROS_ERROR("wallFollower: Could not find control/w0 parameter!");
	ROS_INFO("wallFollower: Loaded Parameter\n w0: %f", w0);
	if (!nhp.getParam("control/sensorTreshold", sensorTreshold)) ROS_ERROR("wallFollower: Could not find control/sensorTreshold parameter!");
	ROS_INFO("wallFollower: Loaded Parameter\n sensorTreshold: %f", sensorTreshold);
}
// Function definitions
void Listener::callbackSensor(const interfaces::Sensor::ConstPtr& msg_in)
{
	msg_sensor = *msg_in;
}
void Listener::callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in)
{
	msg_odometry = *msg_in;
}
void Listener::callbackBumper(const interfaces::Bumper::ConstPtr& msg_in)
{
	msg_bumper = *msg_in;
}
interfaces::Control Listener::getMotorCom() 
{
	// Parameters
	int M = 30;

	// count up
	count = count + 1;

	// Evaluate sensor data
	float r = 0.0f;
	float l = 0.0f;
	if (msg_sensor.right > sensorTreshold) { r = 1.0f; }
	else { r = 0.0f; }
	if (msg_sensor.left > sensorTreshold) { l = 1.0f; }
	else { l = 0.0f; }

	// Start w
	float v = 0.0f;
	float w = 0.0f;

	// Start finding the border line
	if ((l > 0 || r > 0) && !wallFollowing) {
		v = v0;
	}
	else if (!(l>0) && !(r>0)) {
		wallFollowing = true;
	}

	// Do the wall following
	if (wallFollowing) {
		mean = 0.9f * mean + 0.1f * r;
		float d = (0.5f - mean) * 2.0f;
		float var = cosf(2 * PI * ((float)count / (float)M));
		w = (d + var) * 0.5 * w0;
		vCurrent = 0.9f * vCurrent + 0.1f * (1 - fabs(d));
		v = vCurrent * v0;
	}

	// Return the message
	interfaces::Control msg_out;
	msg_out.v = v;
	msg_out.w = w;
	return msg_out;
}

int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "wallFollowing");
  // Initialize node handle
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Initialize the publisher for the control commands, tell them which message should be published
  ros::Publisher simple_auto_pub = nh.advertise<interfaces::Control>("controlData", 1000);
  // Initialize the subscriber to get the teleop data
  Listener listener(nhp);
  ros::Subscriber sub_sensor = nh.subscribe("sensorData", 1000, &Listener::callbackSensor, &listener);
  ros::Subscriber sub_odometry = nh.subscribe("odometryData", 1000, &Listener::callbackOdometry, &listener);
  ros::Subscriber sub_bumper = nh.subscribe("bumperData", 1000, &Listener::callbackBumper, &listener);
  // Loop rate
  double frequency;
  if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("wallFollower: Could not find system/frequency parameter!");
  ros::Rate loop_rate(frequency);
  // Start the loop, but before just wait a few seconds until all sensors are ready to go
  ros::Duration(5).sleep();
  while (ros::ok())
  {
	// Subscribe using the callback functions
	ros::spinOnce();
	// Initialize increment message
	interfaces::Control msg;
	// Calculate Increments
	msg = listener.getMotorCom();
    // publish message
	simple_auto_pub.publish(msg);
	// Sleep until rate is reached
    loop_rate.sleep();
  }
  return 0;
}
