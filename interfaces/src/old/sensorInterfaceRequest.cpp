/*
 * This node enables a connection between ROS and a serial port and 
 * reads data from the serial port.
 * 
 * Be sure that the correct serial port is chosen!
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <interfaces/Sensor.h>

// Initialize serial holder
serial::Serial serRight;
serial::Serial serLeft;

// Fucntion to transfrom string into sensor message
interfaces::Sensor analyze(std_msgs::String msgRight, std_msgs::String msgLeft)
{	
	// Transform string to uint8_t
	interfaces::Sensor msg_out;
	msg_out.r1 = reinterpret_cast<uint8_t>((unsigned char)msgRight.data.c_str()[0]);
	msg_out.l1 = reinterpret_cast<uint8_t>((unsigned char)msgLeft.data.c_str()[0]);
	return msg_out;
}


int main (int argc, char** argv){
	// initialize the node
    ros::init(argc, argv, "sensorInterface");
    ros::NodeHandle nh;								// public handle for topics, etc.
	ros::NodeHandle nhp("~");						// private handle to get parameters
    ROS_INFO("Sensor interface is initialized!");
	// define publisher
    ros::Publisher sensor_pub = nh.advertise<interfaces::Sensor>("sensorData", 1000);
	// Try to initialize the right serial connection and catch errors
    try
    {
        serRight.setPort("/dev/sensor03");
        serRight.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serRight.setTimeout(to);
        serRight.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR("Unable to open right serial port!");
        return -1;
    }
	// Check if serial port is open
    if(serRight.isOpen()){
        ROS_INFO("Right serial port initialized!");
    }else{
		ROS_ERROR("Right serial Port not open!");
        return -1;
    }
	// Try to initialize the left serial connection and catch errors
	try
	{
		serLeft.setPort("/dev/sensor02");
		serLeft.setBaudrate(9600);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		serLeft.setTimeout(to);
		serLeft.open();
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR("Unable to open left serial port!");
		return -1;
	}
	// Check if serial port is open
	if (serRight.isOpen()) {
		ROS_INFO("Left serial port initialized!");
	}
	else {
		ROS_ERROR("Left serial Port not open!");
		return -1;
	}
	// Standard message to sensor
	std_msgs::String act;
	act.data = "d";
	// Start receiving the messages and publish them
	double frequency;
	if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("Could not find system/frequency parameter!");
	ros::Rate loop_rate(frequency);
    while(ros::ok()){
		// Get data from the sensors
		serRight.write(act.data);
		serLeft.write(act.data);
        if(serRight.available() && serLeft.available()) {	
			// Create standard string message
            std_msgs::String resultRight;
			std_msgs::String resultLeft;
			// read data from serial port
			resultRight.data = serRight.read(serRight.available());
			resultLeft.data = serLeft.read(serLeft.available());
			// Transform data
			interfaces::Sensor result_tr = analyze(resultRight, resultLeft);
			// publish data
			sensor_pub.publish(result_tr);
        }     
        loop_rate.sleep();
    }
}

