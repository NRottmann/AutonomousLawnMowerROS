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
#include <interfaces/Decawave.h>

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>

static void error(const char *msg)
{
	perror(msg);
	exit(0);
}

int main (int argc, char** argv){
	// initialize the node
    ros::init(argc, argv, "decawaveInterface");
    ros::NodeHandle nh;								// public handle for topics, etc.
	ros::NodeHandle nhp("~");						// private handle to get parameters
    ROS_INFO("Decawave interface is initialized!");

	sleep(5);

	// Get a publisher
	ros::Publisher decawave_pub = nh.advertise<interfaces::Decawave>("decawaveData", 1000);

	std::string device = "/dev/sensor03";
	// Connection to anchor device
	int serialfd = open(device.c_str(), O_RDWR | O_NONBLOCK | O_NDELAY);
	if (serialfd < 0)
		error("ERROR opening serial device");
	std::cout << "Successfully open the device!\n";
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	tty.c_iflag = 0;					// from here from Ralf
	tty.c_oflag = 0;
	tty.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
	tty.c_lflag = 0;
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 5;
	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);
	if (tcsetattr(serialfd, TCSANOW, &tty) != 0)
		error("Error accessing terminal parameters");
	std::cout << "Successfully setting terminal parameters!\n";

	std::cout << "Initializing anchor device\n";
	unsigned char cmd[] = { '\r','l','e','p','\r' };
	usleep(10000);
	write(serialfd, cmd, 1);
	usleep(10000);
	write(serialfd, cmd, 1);
	sleep(2);
	int length; unsigned char x;
	do {
		length = read(serialfd, &x, 1);
		if (length == 1) std::cout << x;
	} while (length == 1);
	write(serialfd, cmd + 1, 4);
	sleep(2);
	do {
		length = read(serialfd, &x, 1);
		if (length == 1) std::cout << x;
	} while (length == 1);
	std::cout << "Initialization complete\n";
	// Loop until cancelled by user
	std::string serialstring = "";
	double frequency;
	if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("Could not find system/frequency parameter!");
	ros::Rate loop_rate(frequency);
    while(ros::ok()){
		// Process incoming serial data
		int length;
		do {
			unsigned char serialbuf[1];
			length = read(serialfd, serialbuf, 1);
			if (length == 1) {
				// std::cout << serialbuf[0] << "\n";
				if (serialbuf[0] == '\n') {
					if (serialstring.compare(0, 3, "POS") == 0) {
						int pos = serialstring.find(",");
						serialstring.erase(0, pos + 1);
						pos = serialstring.find(",");
						float posX = atof(serialstring.substr(0, pos).c_str());
						serialstring.erase(0, pos + 1);
						pos = serialstring.find(",");
						float posY = atof(serialstring.substr(0, pos).c_str());
						serialstring.erase(0, pos + 1);
						pos = serialstring.find(",");
						float posZ = atof(serialstring.substr(0, pos).c_str());
						serialstring.erase(0, pos + 1);
						float conf = atof(serialstring.c_str());
						serialstring = "";
						// Debugging
						std::cout << "x: " << posX << "  y: " << posY << "  z: " << posZ << "  c: " << conf << "\n";
						interfaces::Decawave msg;
						msg.x = posX;
						msg.y = posY;
						msg.z = posZ;
						msg.confidence = conf;
						decawave_pub.publish(msg);
					}
					serialstring = "";
				}
				else {
					serialstring += serialbuf[0];
				}
			}
		} while (length != 0);
        loop_rate.sleep();
    }
}

