/*
 * This node enables a connection between ROS and a serial port and 
 * reads data from the serial port.
 * 
 * Be sure that the correct serial port is chosen!
 *
 * This node receives radar signals!
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <interfaces/Capacity.h>

#include <string.h>

// Initialize serial holder
serial::Serial ser;

int main (int argc, char** argv){
	// initialize the node
    ros::init(argc, argv, "capacityInterface");
    ros::NodeHandle nh;								// public handle for topics, etc.
	ros::NodeHandle nhp("~");						// private handle to get parameters
    ROS_INFO("Capacity interface is initialized!");
	// define publisher
    ros::Publisher capacity_pub = nh.advertise<interfaces::Capacity>("capacityData", 1000);
	// Try to initialize the right serial connection and catch errors
	std::string sensorID = "/dev/sensor02";
    try
    {
        ser.setPort(sensorID.c_str());
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR("capacityInterface: Unable to open serial port %s!", sensorID.c_str());
        return -1;
    }
	// Check if serial port is open
    if(ser.isOpen()){
        ROS_INFO("capacityInterface: Serial port %s initialized!", sensorID.c_str());
    }else{
		ROS_ERROR("capacityInterface: Serial port %s not open!", sensorID.c_str());
        return -1;
    }	
	// Start receiving the messages and publish them
	int L = 10;
    while(ros::ok()){
		// Get data from the sensors
        while(ser.available() >= L) {
			// Read data from serial port
			unsigned char buffer[L];
			int length = 0;
			while (length < L)
				length += ser.read(buffer + length, L - length);
			interfaces::Capacity msg;
			for (int l = 0; l < L; l = l + 2) {
				msg.data[l/2] = (uint16_t)(buffer[l]) + (uint16_t)(buffer[l + 1] * 256);
			}
			capacity_pub.publish(msg);
        }     
    }
}

