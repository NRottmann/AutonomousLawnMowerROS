#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <interfaces/Control.h>
#include <interfaces/Decawave.h>

// Define a callback class
class DecawaveListener
{
	public:
		void callback(const interfaces::Decawave::ConstPtr& msg_in);
		interfaces::Control getMotorCommands();
	private:
	    interfaces::Decawave msg_store;
		interfaces::Decawave msg_store_old;
		int pos = 0;
};
// Define Functions
// Callback function to get the data from the topic
void DecawaveListener::callback(const interfaces::Decawave::ConstPtr& msg_in)
{
	msg_store_old = msg_store;
	msg_store = *msg_in;
}
// Calculates the desired speed
interfaces::Control DecawaveListener::getMotorCommands() 
{
	double v_max = 0.3;
	double w_max = 0.3;

	double xT[4] = { 0.5, 3.5, 3.5, 0.5 };
	double yT[4] = { 0.5, 0.5, 3.5, 3.5 };

	// Position counter
	double ds = (xT[pos] - msg_store.x)*(xT[pos] - msg_store.x) + (yT[pos] - msg_store.y)*(yT[pos] - msg_store.y);
	if (ds < 0.5) {
		if (pos == 3) {
			pos = 0;
		}
		else {
			pos = pos + 1;
		}
	}

	// Get the actual orientation
	double dx = msg_store.x - msg_store_old.x;
	double dy = msg_store.y - msg_store_old.y;
	double phi = atan2(dy, dx);

	// Calculate Motor Commands
	double e_x = msg_store.x - xT[pos];
	double e_y = msg_store.y - yT[pos];
	double e_vx = dx / 0.01;
	double e_vy = dy / 0.01;

	double F_x = -e_x - e_vx;
	double F_y = -e_y - e_vy;

	double u1 = F_x * cos(phi) + F_y * sin(phi);

	double phi_des = atan2(-e_y, -e_x);
	
	double u2 = phi_des - phi;
	
	// Allocate motor control to message
	interfaces::Control msg_control;
	if (u1 > v_max) {
		u1 = v_max;
	}
	else if (u1 < -v_max) {
		u1 = -v_max;
	}
	if (u2 > w_max) {
		u2 = w_max;
	}
	else if (u2 < -w_max) {
		u2 = -w_max;
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
  ros::Publisher tele_pub = nh.advertise<interfaces::Control>("controlData", 1000);
  // Initialize the subscriber to get the teleop data
  DecawaveListener listener;
  ros::Subscriber sub = nh.subscribe("decawaveData", 1000, &DecawaveListener::callback, &listener);
  // Loop rate
  double frequency;
  if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("Could not find system/frequency parameter!");
  ros::Rate loop_rate(frequency);
  // Start the loop
  while (ros::ok())
  {
	// Initialize control message
	interfaces::Control msg;
	// Calculate Increments
	msg = listener.getMotorCommands();
    // publish message
	tele_pub.publish(msg);
	// Subscribe using the callback functions
    ros::spinOnce();
	// Sleep until rate is reached
    loop_rate.sleep();
  }
  return 0;
}
