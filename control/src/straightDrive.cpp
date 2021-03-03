#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <interfaces/Control.h>
#include <interfaces/Odometry.h>

// Define a callback class
class StraightDrive
{
public:
        StraightDrive(ros::NodeHandle nh, ros::NodeHandle nhp);
	void callback(const interfaces::Odometry::ConstPtr& msg_in);
	void publishMotorcommands(float v);
        void actualizePosition();
private:
	// Parameter
	float L = 0.1788;

	// Current message
	interfaces::Odometry msg_store;

	// Current positions
	float x;
	
	// Subscriber and Publisher
	ros::Subscriber sub;
	ros::Publisher pub;
};

// Constructor
StraightDrive::StraightDrive(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Get Parameters
	if (!nhp.getParam("robot/L", L)) ROS_ERROR("straightDrive: Could not find robot/L parameter!");
	ROS_INFO("straightDrive: Loaded Parameter\n L: %f", L);
	// Initialize subscriber and publisher
	sub = nh.subscribe("odometryData", 20, &StraightDrive::callback, this);
	pub = nh.advertise<interfaces::Control>("controlData", 20);
	// Initialize variables
	x = 0.0f;
}
void StraightDrive::callback(const interfaces::Odometry::ConstPtr& msg_in)
{
	ROS_INFO("%f", x);
	msg_store = *msg_in;
	actualizePosition();
	if (x > 3.0f) {
		ROS_INFO("Shut down!");
		publishMotorcommands(0.0f);
		ros::shutdown();
	} else {
		publishMotorcommands(0.3f);
	}
}
// Actualize position
void StraightDrive::actualizePosition()
{	
	float ds = 0.5 * (msg_store.l_R + msg_store.l_L);
	if(std::fabs(ds) < 0.2) {
		x = x + ds; 
	} else {
		x = x;
	}
}
void StraightDrive::publishMotorcommands(float v) {
	interfaces::Control msg_out;
	msg_out.v = v;
	msg_out.w = 0;
	pub.publish(msg_out);
}

int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "straightDrive");
  // Initialize node handle
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Initialize the subscriber to get the teleop data
  StraightDrive straightDrive(nh, nhp);
  // Wait until we drove fare enough, then end ros
  ros::spin();
  return 0;
}
