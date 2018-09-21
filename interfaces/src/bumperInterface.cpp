/*
 * This node enables a connection between ROS and a serial port and 
 * reads data from the serial port.
 * 
 * Be sure that the correct serial port is chosen!
 */

#include <ros/ros.h>
#include <interfaces/IMU.h>
#include <interfaces/Bumper.h>

 // Define a callback class for the motor commands
class IMUBumper
{
public:
	void callback(const interfaces::IMU::ConstPtr& msg_in);
	interfaces::Bumper getBumperMsg();
	IMUBumper(ros::NodeHandle nhp);		// This is the constructor, which loads the variables
private:
	interfaces::IMU msg_store;
	float bumperTreshold = 0.0f;
};
// Define Functions for class ControlListener
IMUBumper::IMUBumper(ros::NodeHandle nhp)
{
	if (!nhp.getParam("control/bumperTreshold", bumperTreshold)) ROS_ERROR("Could not find control/bumperTreshold parameter!");
	ROS_INFO("Bumper Interface: Loaded Control Parameter\n bumperTreshold: %f", bumperTreshold);
}
void IMUBumper::callback(const interfaces::IMU::ConstPtr& msg_in)
{
	msg_store = *msg_in;
}
interfaces::Bumper IMUBumper::getBumperMsg() {
	interfaces::Bumper msg_out;

	// Ralfs implementation
	/* float treshold = bumperTreshold * bumperTreshold;
	float a = msg_store.linear_acceleration.x * msg_store.linear_acceleration.x + msg_store.linear_acceleration.y * msg_store.linear_acceleration.y;
	if ((msg_store.linear_acceleration.y) > bumperTreshold) {
		msg_out.bump = true;
		msg_out.phi = msg_store.linear_acceleration.x;
	}
	else {
		msg_out.bump = false;
	} */

	float treshold = bumperTreshold * bumperTreshold;
	float a = msg_store.linear_acceleration.x * msg_store.linear_acceleration.x + msg_store.linear_acceleration.y * msg_store.linear_acceleration.y;
	if (a > treshold) {
		msg_out.bump = true;
		msg_out.phi = atan2f(-msg_store.angular_velocity.x, -msg_store.angular_velocity.y);
	}
	else {
		msg_out.bump = false;
	}
	return msg_out;
}


int main (int argc, char** argv){
	// initialize the node
    ros::init(argc, argv, "bumperInterface");
    ros::NodeHandle nh;								// public handle for topics, etc.
	ros::NodeHandle nhp("~");						// private handle to get parameters
    ROS_INFO("Bumper interface is initialized!");
	// define publisher
    ros::Publisher bumper_pub = nh.advertise<interfaces::Bumper>("bumperData", 1000);
	// Initialize class
	IMUBumper bumper(nhp);
	ros::Subscriber imu_sub = nh.subscribe("imuData", 1000, &IMUBumper::callback, &bumper);
	// Start receiving the messages and publish them
	double frequency;
	if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("Could not find system/frequency parameter!");
	ros::Rate loop_rate(frequency);
    while(ros::ok()){
		// Get IMU data, check if a bump happenend and publsih bumper data
		ros::spinOnce();
		interfaces::Bumper msg;
		msg = bumper.getBumperMsg();
		bumper_pub.publish(msg);
        loop_rate.sleep();
    }
}

