#include "ros/ros.h"
#include "std_msgs/String.h"
#include <interfaces/Odometry.h>
#include <interfaces/Control.h>

#include <signal.h>						// Packages for shutdown commands
#include <ros/xmlrpc_manager.h>

#include <sstream>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

// Here we replace the SIGINT handle in order to be able to give some last commands after shut down ROS
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
	g_request_shutdown = 1;
}
// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
	int num_params = 0;
	if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
		num_params = params.size();
	if (num_params > 1)
	{
		std::string reason = params[1];
		ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
		g_request_shutdown = 1; // Set flag
	}
	result = ros::xmlrpc::responseInt(1, "", 0);
}

// Define that as a global structure, otherwiese we can not send more then one byte
struct spi_ioc_transfer tr[256];

// Define a callback class for the motor commands
class ControlListener
{
	public:
		void callback(const interfaces::Control::ConstPtr& msg_in);
		void calculateMotorCom(unsigned char *msg_out_right, unsigned char *msg_out_left);
		ControlListener(ros::NodeHandle nhp);		// This is the constructor, which loads the variables
	private:
		interfaces::Control msg_store;
		// Parameters
		float L = 0.0f;					// half of the wheelbase, in m
		float maxSpeed = 0.0f;			// maximum speed what can be achieved by the vehicle, in m/s
		int maxTicksSpeed = 0;			// Maximum Number of steps in which the speed can be divided, 0 = 0 m/s, maxTickSpeed = maxSpeed
		float vChange = 0.0f;
		float wChange = 0.0f;
		float v0 = 0.0f;				// Old velocities
		float w0 = 0.0f;
};
// Define Functions for class ControlListener
ControlListener::ControlListener(ros::NodeHandle nhp)
{
	if (!nhp.getParam("robot/L", L)) ROS_ERROR("Could not find robot/L parameter!");
	if (!nhp.getParam("robot/maxSpeed", maxSpeed)) ROS_ERROR("Could not find robot/maxSpeed parameter!");
	if (!nhp.getParam("robot/maxTicksSpeed", maxTicksSpeed)) ROS_ERROR("Could not find robot/maxTicksSpeed parameter!");
	ROS_INFO("Motor Interface: Loaded Robot Parameter\n L: %f, maxSpeed %f, maxTicksSpeed: %i", L, maxSpeed, maxTicksSpeed);
	if (!nhp.getParam("control/vChange", vChange)) ROS_ERROR("Could not find control/vChange parameter!");
	if (!nhp.getParam("control/wChange", wChange)) ROS_ERROR("Could not find control/wChange parameter!");
	ROS_INFO("Motor Control Interface: Loaded Parameter\n vChange: %f, wChange: %f", vChange, wChange);
}
void ControlListener::callback(const interfaces::Control::ConstPtr& msg_in)
{
	msg_store = *msg_in;
}
void ControlListener::calculateMotorCom(unsigned char *msg_out_right, unsigned char *msg_out_left)
{
	// Normalize onto maximum if required
	float cumSpeed = abs(msg_store.v) + abs(msg_store.w)*L;
	if (cumSpeed > maxSpeed) {
		msg_store.v = msg_store.v * maxSpeed / cumSpeed;
		msg_store.w = msg_store.w * maxSpeed / cumSpeed;
	}
	// Use a ramp for slow acceleration
	float v = 0.0f;
	float w = 0.0f;
	float vDiff = msg_store.v - v0;
	if (vDiff > vChange) {
		v = v0 + vChange;
	}
	else if (vDiff < -vChange) {
		v = v0 - vChange;
	}
	else {
		v = msg_store.v;
	}
	float wDiff = msg_store.w - w0;
	if (wDiff > wChange) {
		w = w0 + wChange;
	}
	else if (wDiff < -wChange) {
		w = w0 - wChange;
	}
	else {
		w = msg_store.w;
	}
	v0 = v;
	w0 = w;
	// Calculate wheel speed
	float speedRight = v + L * w;
	float speedLeft = w * L - v;
	// check the signs and assign directions
	bool rightDirection = speedRight >= 0;
	bool leftDirection = speedLeft >= 0;
	// Set speed
	float rightSpeed = abs(speedRight);
	float leftSpeed = abs(speedLeft);
	// Transform message
	if (rightDirection) {
		msg_out_right[0] = 0x11;
	} else {
		msg_out_right[0] = 0x12;
	}
	if (leftDirection) {
		msg_out_left[0] = 0x11;
	} else {
		msg_out_left[0] = 0x12;
	}
	// Calculate bytes to send to the motor
	uint8_t velComIntRight = 0;
	uint8_t velComIntLeft = 0;
	float velComRight = (rightSpeed * maxTicksSpeed);
	float velComLeft = (leftSpeed * maxTicksSpeed);

	if (velComRight > 255) {
		velComIntRight = 255;
	} else if (velComRight <= 0) {
		velComIntRight = 0;
		msg_out_right[0] = 0x00;
	} else {
		velComIntRight = (uint8_t)(velComRight);
	}
	msg_out_right[1] = velComIntRight;

	if (velComLeft > 255) {
		velComIntLeft = 255;
	} else if (velComLeft <= 0) {
		velComIntLeft = 0;
		msg_out_left[0] = 0x00;
	} else {
		velComIntLeft = (uint8_t)(velComLeft);
	}
	// Give message back
	msg_out_left[1] = velComIntLeft;
}

// Class to handle the odometry messages from the motor
class OdometryEvaluation
{
public:
	void actPosition(uint16_t posRight, uint16_t posLeft);
	void actCurrent(uint16_t cRight, uint16_t cLeft);
	void actSpeed(int16_t vRight, int16_t vLeft);
	float getDistance(uint16_t act, uint16_t past);
	interfaces::Odometry getOdometry();
	OdometryEvaluation(ros::NodeHandle nhp);
private:
	uint16_t pastPosRight = 0;			// position of the wheels in ticks measured by Hall Sensor A
	uint16_t pastPosLeft = 0;
	uint16_t actPosRight = 0;
	uint16_t actPosLeft = 0;
	uint16_t currentRight = 0;			// Current though the motor
	uint16_t currentLeft = 0;
	int16_t vR = 0;						// Velocity of the wheels calculated by the motor
	int16_t vL = 0;
	// Parameter
	float nHall = 0.0f;			// number of Ticks counted by Hall Sensor A per round of the wheel
	float d = 0.0f;				// diameter of a wheel, in m
};
OdometryEvaluation::OdometryEvaluation(ros::NodeHandle nhp)
{
	if (!nhp.getParam("robot/d", d)) ROS_ERROR("Could not find robot/d parameter!");
	if (!nhp.getParam("robot/nHall", nHall)) ROS_ERROR("Could not find robot/nHall parameter!");
	ROS_INFO("Motor Interface: Loaded Parameter\n d: %f, nHall %f", d, nHall);
}
void OdometryEvaluation::actPosition(uint16_t posRight, uint16_t posLeft)
{
	pastPosRight = actPosRight;
	pastPosLeft = actPosLeft;
	actPosRight = posRight;
	actPosLeft = posLeft;
}
void OdometryEvaluation::actCurrent(uint16_t cRight, uint16_t cLeft)
{
	currentRight = cRight;
	currentLeft = cLeft;
}
void OdometryEvaluation::actSpeed(int16_t vRight, int16_t vLeft)
{
	vR = vRight;
	vL = vLeft;
}
float OdometryEvaluation::getDistance(uint16_t act, uint16_t past)
{
	// Define Motor Constants							
	float lengthPerTick = d * 3.1415 / nHall;
	int treshold = 10*nHall;						// treshold for overshooting
	int maxCount = 65535;							// maximum Count of position ticks, these are just 2 Bytes
	// distance driven by the wheels
	int diff = act - past;
	float dis = 0.0f;
	if (diff > treshold) // negative overshooting happens	
	{
		dis = -(past + maxCount - act) * lengthPerTick;
	}
	else if (diff < -treshold) // positive overshooting happens	
	{
		dis = (act + maxCount - past) * lengthPerTick;
	}
	else {
		dis = diff * lengthPerTick;
	}
	return dis;
}
interfaces::Odometry OdometryEvaluation::getOdometry()
{
	interfaces::Odometry msg;	
	msg.l_R = getDistance(actPosRight, pastPosRight);
	msg.l_L = -getDistance(actPosLeft, pastPosLeft);
	msg.i_R = currentRight;
	msg.i_L = currentLeft;
	msg.v_R = vR;
	msg.v_L = -vL;
	msg.rawR = actPosRight;
	msg.rawL = actPosLeft;

	return msg;

	// ROS_INFO("l_R: %f,  l_L: %f", msg.l_R, msg.l_L);
}

// Function to read and write via the SPI connection with the motor
void SpiWriteRead(int length, const char *device, unsigned char *msg)
{
	// Initialize parameters
	uint8_t mode = 0;
	uint8_t bits = 8;
	uint32_t speed = 32000;
	// Open device
	int fd = open(device, O_RDWR);
	if (fd < 0) { 
		ROS_ERROR("Can't open device"); 
		exit(1);
	}
	// SPI mode
	int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret < 0) { 
		ROS_ERROR("Can't set spi mode");
		exit(1);
	}
	// Bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret < 0) { 
		ROS_ERROR("Can't set bits per word");
		exit(1);
	}
	// Max speed, 1ms per 32bit command --> 32kHz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret < 0) { 
		ROS_ERROR("Can't set max speed"); 
		exit(1);
	}
	// Initialize holders with enough buffer
  	unsigned char tx[256];
  	unsigned char rx[256];
  	for (int i = 0; i < length; i++) {
	  	tx[i] = msg[i];
	  	rx[i] = 0x00;
  	}
	for (int i = 0; i < length; i++) {
		tr[i].tx_buf = (unsigned long)&tx[i];
		tr[i].rx_buf = (unsigned long)&rx[i];
		tr[i].len = 1;
		tr[i].delay_usecs = 1000;
		tr[i].speed_hz = speed;
		tr[i].bits_per_word = 8;
	}
	ret = ioctl(fd, SPI_IOC_MESSAGE(length), tr);
	if (ret < 1)
	{
		ROS_ERROR("Error read or write - ioctl");
		exit(1);
	}
	// Give back the received message
	for (int i = 0; i < length; i++) {
		msg[i] = rx[i];
	}
	// Close device
	close(fd);
}


int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "motorInterface", ros::init_options::NoSigintHandler);	// Initialize this node to with noSigintHandle to add an own one
  signal(SIGINT, mySigIntHandler);
  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
  // Initialize node handles
  ros::NodeHandle nh;						// public node handle, to publish topics etc.
  ros::NodeHandle nhp("~");					// private node handle, to get parameters
  ROS_INFO("Motor Interface is initialized!");
  // Ros rate
  // ros::Duration duration(1. / 100.);
  double frequency;
  if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("Could not find system/frequency parameter!");
  ros::Rate loop_rate(frequency);
  // Define publisher, TODO: add third motor
  OdometryEvaluation odoEva(nhp);
  ros::Publisher odometry_pub = nh.advertise<interfaces::Odometry>("odometryData", 1000);
  // Initialize the subscribers, TODO: add third motor
  ControlListener listenerControl(nhp);
  ros::Subscriber control_sub = nh.subscribe("controlData", 1000, &ControlListener::callback, &listenerControl);
  // Initialize messages with zeros and device handles
  // Length of byte stream
  int length = 7;
  unsigned char rightMsg[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char leftMsg[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  const char *dev0 = "/dev/spidev1.0";
  const char *dev1 = "/dev/spidev1.1";
  // Wait a few seconds before everything is initialized
  sleep(2);
  // Start the ROS loop and proceed until shutdown is requested
  while (!g_request_shutdown)
  {
	  // Subscribe using the callback functions
	  ros::spinOnce();
	  // Calculate the motor commands from the messages
	  listenerControl.calculateMotorCom(rightMsg, leftMsg);
	  // Send data to SPI and read simultanous
	  SpiWriteRead(length, dev0, rightMsg);
	  SpiWriteRead(length, dev1, leftMsg);
	  // Create messages for publishing them
	  interfaces::Odometry odo;
	  // Odometry Evaluation
	  odoEva.actPosition((uint16_t)((rightMsg[1] << 8) | rightMsg[2]), (uint16_t)((leftMsg[1] << 8) | leftMsg[2]));
	  odoEva.actCurrent((uint16_t)((rightMsg[3] << 8) | rightMsg[4]), (uint16_t)((leftMsg[3] << 8) | leftMsg[4]));
	  odoEva.actSpeed((int16_t)((rightMsg[5] << 8) | rightMsg[6]), (int16_t)((leftMsg[5] << 8) | leftMsg[6]));
	  odo = odoEva.getOdometry();
	  // Add time data
	  odo.header.stamp = ros::Time::now();
	  // publish the data
	  odometry_pub.publish(odo);
	  // Sleep until rate is reached
	  //duration.sleep();
	  loop_rate.sleep();
  }
  // Do pre-shutdown tasks
  // Set motor velocities to zero
  rightMsg[0] = 0x00;
  rightMsg[1] = 0x00;
  leftMsg[0] = 0x00;
  leftMsg[1] = 0x00;
  SpiWriteRead(length, dev0, rightMsg);
  SpiWriteRead(length, dev1, leftMsg);
  ROS_INFO("Shutdown commands for Motor Interface successfully executed! Shut down ROS!");
  ros::shutdown();
}
