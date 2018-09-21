#include "ros/ros.h"
#include "std_msgs/String.h"
#include <interfaces/Control.h>
#include <localization/Pose.h>

#define PI 3.14159265f

// Define a callback class
class PDListener
{
	public:
		void callback(const localization::Pose::ConstPtr& msg_in);
		interfaces::Control getMotorCommands();
		PDListener(ros::NodeHandle nh, ros::NodeHandle nhp);						// this is the constructor
	private:
	    localization::Pose msg_pose;
		localization::Pose msg_pose_old;
		// Parameters
		float Pv, Pw, Dv, Dw;
		float v0, w0;
		// Position counter
		int pos;
		// Subscriber
		ros::Subscriber sub;
};
// Constructor
PDListener::PDListener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Get Parameters
	if (!nhp.getParam("control/Pv", Pv)) ROS_ERROR("pdControl: Could not find control/Pv parameter!");
	ROS_INFO("pdControl: Loaded Parameter\n Pv: %f", Pv);
	if (!nhp.getParam("control/Pw", Pw)) ROS_ERROR("pdControl: Could not find control/Pw parameter!");
	ROS_INFO("pdControl: Loaded Parameter\n Pw: %f", Pw);
	if (!nhp.getParam("control/Dv", Dv)) ROS_ERROR("pdControl: Could not find control/Dv parameter!");
	ROS_INFO("pdControl: Loaded Parameter\n Dv: %f", Dv);
	if (!nhp.getParam("control/Dw", Dw)) ROS_ERROR("pdControl: Could not find control/Dw parameter!");
	ROS_INFO("pdControl: Loaded Parameter\n Dw: %f", Dw);
	if (!nhp.getParam("control/v0", v0)) ROS_ERROR("randomWalk: Could not find control/v0 parameter!");
	ROS_INFO("randomWalk: Loaded Parameter\n v0: %f", v0);
	if (!nhp.getParam("control/w0", w0)) ROS_ERROR("randomWalk: Could not find control/w0 parameter!");
	ROS_INFO("randomWalk: Loaded Parameter\n w0: %f", w0);
	// Initialize subscriber
	sub = nh.subscribe("kalmanFilterPose", 1000, &PDListener::callback, this);
	// Initialize variables
	pos = 0;
}
// Callback function to get the data from the topic
void PDListener::callback(const localization::Pose::ConstPtr& msg_in)
{
	msg_pose_old = msg_pose;
	msg_pose = *msg_in;
}
// Calculates the desired speed
interfaces::Control PDListener::getMotorCommands()
{
	// Positions to visit
	float xT[4] = { 0.0, 2.0, 2.0, 0.0 };
	float yT[4] = { 0.0, 0.0, 2.0, 2.0 };

	// Check whether we are already near enough the desired position
	float ds = (xT[pos] - msg_pose.pose[0])*(xT[pos] - msg_pose.pose[0]) + (yT[pos] - msg_pose.pose[1])*(yT[pos] - msg_pose.pose[1]);
	if (ds < 0.5) {
		if (pos == 3) {
			pos = 0;
		}
		else {
			pos = pos + 1;
		}
	}

	// Calculate Motor Commands, TODO: add D-part
	float dt_n = (((float)msg_pose.header.stamp.nsec - (float)msg_pose_old.header.stamp.nsec) / 1000000000.0f);
	if (dt_n < 0)
		(dt_n = 1.0f - dt_n);
	float dt = ((float)msg_pose.header.stamp.sec - (float)msg_pose_old.header.stamp.sec) + dt_n;
	float e_x = xT[pos] - msg_pose.pose[0];
	float e_y = yT[pos] - msg_pose.pose[1];
	float u1 = Pv * (cosf(msg_pose.pose[2]) * e_x + sinf(msg_pose.pose[2]) * e_y);

	float phi_des = atan2f(e_y, e_x);
	float phi_act = msg_pose.pose[2] - 2 * PI*floorf(msg_pose.pose[2] / (2 * PI));
	if (phi_act > PI)
		phi_act = phi_act - 2 * PI;
	float dphi = phi_des - phi_act;
	if (dphi < -PI)
		dphi = 2*PI + dphi;
	if (dphi > PI)
		dphi = dphi - 2 * PI;
	float u2 = Pw * dphi;

	ROS_INFO("phi: /t %f", phi_act);
	ROS_INFO("dphi: /t %f", dphi);
	ROS_INFO("p_act: /t %f, /t %f, /t %f", msg_pose.pose[0], msg_pose.pose[1], msg_pose.pose[2]);
	ROS_INFO("p_des: /t %f, /t %f, /t %f", xT[pos], yT[pos], phi_des);
	
	// Allocate motor control to message
	interfaces::Control msg_control;
	if (u1 > v0) {
		u1 = v0;
	}
	else if (u1 < 0) {
		u1 = 0;
	}
	if (u2 > w0) {
		u2 = w0;
	}
	else if (u2 < -w0) {
		u2 = -w0;
	}

	msg_control.v = u1;
	msg_control.w = u2;

	return msg_control;
}


int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "decawaveControl");
  // Initialize node handle
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Initialize the publisher for the control commands, tell them which message should be published
  ros::Publisher pub = nh.advertise<interfaces::Control>("controlData", 1000);
  // Initialize the subscriber to get the teleop data
  PDListener pdlistener(nh, nhp);
  // Loop rate
  double frequency;
  if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("Could not find system/frequency parameter!");
  ros::Rate loop_rate(frequency);
  // Start the loop
  while (ros::ok())
  {
	// Subscribe using the callback functions
	ros::spinOnce();
	// Initialize control message
	interfaces::Control msg;
	// Calculate Increments
	msg = pdlistener.getMotorCommands();
    // publish message
	pub.publish(msg);
	// Sleep until rate is reached
    loop_rate.sleep();
  }
  return 0;
}
