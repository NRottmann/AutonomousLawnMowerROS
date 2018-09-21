#include "ros/ros.h"
#include <interfaces/Control.h>
#include <interfaces/Odometry.h>
#include <interfaces/Sensor.h>
#include <interfaces/Bumper.h>
#include <math.h>

#define PI 3.14159265

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
		bool r, l;						// Boolean for sensor answer
		float L = 0.0f;					// Half the distance between the wheels
		int firstSensor = 0;			// shows which sensor first does not detect any grass, 0: all sensors detect grass, 1: r has not detected first, 2: left has not detected first
		float edgeDistance = 0.0f;		
		bool turn = false;				// true->the vehicle turns
		float turned = 0;				// Here we cumulate how much we already turned according to the odometry data
		float backward = 0;				// here we cumulate how much we already driven backwards	
		float desTurn = 0;				// desired turning angle
		int count = 0;
		float v0;
		float w0;
		int sensorTreshold;
};
// Constructor
Listener::Listener(ros::NodeHandle nhp) {
	if (!nhp.getParam("robot/L", L)) ROS_ERROR("randomWalk: Could not find robot/L parameter!");
	ROS_INFO("randomWalk: Loaded Parameter\n L: %f", L);
	if (!nhp.getParam("control/v0", v0)) ROS_ERROR("randomWalk: Could not find control/v0 parameter!");
	ROS_INFO("randomWalk: Loaded Parameter\n v0: %f", v0);
	if (!nhp.getParam("control/w0", w0)) ROS_ERROR("randomWalk: Could not find control/w0 parameter!");
	ROS_INFO("randomWalk: Loaded Parameter\n w0: %f", w0);
	if (!nhp.getParam("control/sensorTreshold", sensorTreshold)) ROS_ERROR("randomWalk: Could not find control/sensorTreshold parameter!");
	ROS_INFO("randomWalk: Loaded Parameter\n sensorTreshold: %f", sensorTreshold);
	// initialize random seed:
	srand(time(NULL));
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
	// Define parameters
	float meanMeasurementDistance = 0.15;		// Measurement Distance until we decide wether we really have nothing detected [in metre]

	// Evaluate sensor data
	if (msg_sensor.l1 > sensorTreshold) { l = true; }
	else { l = false; }
	if (msg_sensor.r1 > sensorTreshold) { r = true; }
	else { r = false; }

	// Count for debugging
	count = count + 1;
	if (count > 20) {
		count = 0;
		ROS_INFO("Sensor Left: \t %i", msg_sensor.l1);
		ROS_INFO("Sensor Right: \t %i", msg_sensor.r1);
	}

	// Generate motor message
	float v = 0.0f;
	float w = 0.0f;

	// if bool turn is set to true, we turn the vehicle
	if (turn) {
		if (backward < meanMeasurementDistance) {
			v = -v0;
			backward = backward - (msg_odometry.l_R - msg_odometry.l_L) / 2;
		}
		else {
			if (firstSensor == 1) { w = w0; }
			else { w = -w0; }
			turned = turned + (msg_odometry.l_R + msg_odometry.l_L) / (2 * L);
			if (abs(turned) >= desTurn) {
				turn = false;
				firstSensor = 0;
				edgeDistance = 0.0f;
				turned = 0;
				backward = 0;
			}
		}
	}
	else if (msg_bumper.bump) {
		turn = true;		// the vehicle will now turn
		ROS_INFO("Turning: Bumper");
		desTurn = (rand() / ((float)(RAND_MAX))) * 0.5f * (float)PI + 0.25f * (float)PI - msg_bumper.phi;
	}
	else {
		// if both sensor detect grass, move fwd, reset all parameters
		if (l && r) {
			v = v0;
			// reset variables
			firstSensor = 0;
			edgeDistance = 0.0f;
			turned = 0;
			backward = 0;
		} 
		// otherwise check what is going on
		// If one of the sensors still detects something, then move fwd but tell the program which sensor has first not detect anything
		else if (l && (edgeDistance < meanMeasurementDistance)) {
			firstSensor = 1;
			v = v0;
			// reset
			edgeDistance = edgeDistance + (msg_odometry.l_R - msg_odometry.l_L) * 0.5f;
			turned = 0.0f;
			backward = 0;
		}
		else if (r && (edgeDistance < meanMeasurementDistance)) {
			firstSensor = 2;
			v = v0;
			// reset
			edgeDistance = edgeDistance + (msg_odometry.l_R - msg_odometry.l_L) * 0.5f;
			turned = 0.0f;
			backward = 0;
		}
		// Both sensors detect nothing, since now we might have to turn
		else {
			if (firstSensor == 0) { firstSensor = 1; }
			// Cumulate the distance travelled since none of the sensors had something detect
			edgeDistance = edgeDistance + (msg_odometry.l_R - msg_odometry.l_L) * 0.5f;
			// If the distance is bigger then a certain treshold, we start turning
			if (edgeDistance >= meanMeasurementDistance) {
				turn = true;		// the vehicle will now turn
				ROS_INFO("Turning: Sensor %i", firstSensor);
				desTurn = (rand() / ((float)(RAND_MAX))) * 0.5f * (float)PI + 0.25f * (float)PI;
			}
			else {
				v = v0;
			}
		}
	}

	// Return motor message
	interfaces::Control msg_out;
	msg_out.v = v;
	msg_out.w = w;
	return msg_out;
}

int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "autoDrive");
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
  if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("randomWalk: Could not find system/frequency parameter!");
  ros::Rate loop_rate(frequency);
  // Start the loop
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
