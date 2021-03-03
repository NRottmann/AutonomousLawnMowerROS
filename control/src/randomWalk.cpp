#include "ros/ros.h"
#include <interfaces/Control.h>
#include <interfaces/Odometry.h>
#include <interfaces/Sensor.h>
#include <interfaces/Bumper.h>
#include <math.h>

#define PI 3.14159265f


// Define a callback class
class Listener
{
	public:
		void callbackSensor(const interfaces::Sensor::ConstPtr& msg_in);		// callback function to receive sensor measurements
		void callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in);	// callback function to receive odometry data from the motor
		interfaces::Control getMotorCom();										// function to calculate motor commands
		Listener(ros::NodeHandle nhp);											// this is the constructor
	private:
		interfaces::Sensor msg_sensor;
		interfaces::Odometry msg_odometry;
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
		int backcounter;
		float meanS1 = 1; // Mittelwert oder Fenster der letzten Messwerte
		float meanS2 = 1; // Mittelwert oder Fenster der letzten Messwerte
		float speed = 0; // Robotergeschwindigkeit
		float angleDiff = 0;
		unsigned char mode = 0;
		float decisionMeanSValue = 0.8f;
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

float min (float a, float b)
{
	if (a < b)
		return a;
	else
		return b;
}

interfaces::Control Listener::getMotorCom()
{
	// Return variable
	interfaces::Control msg_out;

	// Process input data
	int S1mess = 0, S2mess = 0;
	if (msg_sensor.left > sensorTreshold) { S1mess = 1; }
	if (msg_sensor.right > sensorTreshold) { S2mess = 1; }

	// Update mean sensor values
	meanS1 = meanS1 * 0.9 + S1mess * 0.1f;
	meanS2 = meanS2 * 0.9 + S2mess * 0.1f;

	// Debug
	// ROS_INFO("meanS1: %f", meanS1);
	// ROS_INFO("meanS2: %f", meanS2);
	// ROS_INFO("mode: %i", mode);

	// Move forward
	if (mode == 0)
	{
        if (meanS1 < decisionMeanSValue || meanS2 < decisionMeanSValue) // Min. ein Sensor macht Mist
			{
                angleDiff = (2*PI/360)*(((meanS1 - meanS2) > 0? 90:-90) - 30 + rand()*60/(float)RAND_MAX);
				msg_out.v = 0;
				msg_out.w = 0;
                mode = 1; // Move back
				if (angleDiff > 0)
					ROS_INFO("Move back for left CSENSOR rotation");
				else
					ROS_INFO("Move back for right CSENSOR rotation");
	   		} else {
                msg_out.w = 0; // Keine Drehung
                msg_out.v = (meanS1 + meanS2 - 2*decisionMeanSValue);//(meanS1 + meanS2)/2.0f; // Max 1 bedeutet 1m/s
            }
    }

	// Move back
	if (mode == 1)
	{
        if ((meanS1 > decisionMeanSValue && meanS2 > decisionMeanSValue) && (backcounter++ > 15))
		{
            msg_out.v = 0;
			msg_out.w = 0;
			mode = 2; // Drehen
			ROS_INFO("Rotate");
        } else {
            msg_out.w = 0;
            msg_out.v = -0.2f;
		}
	}

	// Rotate
	if (mode == 2)
	{
		if (fabs(angleDiff) < 0.1f)
		{
			msg_out.w = 0;
			msg_out.v = 0;
			mode = 0; // Geradeaus, normal
			ROS_INFO("Continue");
        	} else {
			msg_out.w = (angleDiff > 0)?0.5:-0.5;	//((angleDiff > 0?1:-1)*min(0.5f, abs(angleDiff)));
			msg_out.v = 0;
			if (angleDiff > 0)
			{
				angleDiff = angleDiff - min(angleDiff, (msg_odometry.l_R - msg_odometry.l_L) / (2 * L));
			}
			else {
				angleDiff = angleDiff + min(-1*angleDiff, -1*(msg_odometry.l_R - msg_odometry.l_L) / (2 * L));
			}
			//ROS_INFO("angleDiff: %f", angleDiff);
			//angleDiff = angleDiff - msg_out.w;
		}
	}

	// Return motor message
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
