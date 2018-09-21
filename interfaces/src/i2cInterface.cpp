#include "ros/ros.h"
#include "std_msgs/String.h"
// #include <interfaces/SystemControl.h>

#include <sstream>

#include <unistd.h>			// required for I2C port
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// Function to read and write via the SPI connection with the motor
void I2CRead(int length, int device, unsigned char *buffer)
{
	if (read(device, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred
	{
		//ERROR HANDLING: i2c transaction failed
		ROS_ERROR("Failed to read from the i2c bus");
		exit(1);
	}
}


int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "i2cInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ROS_INFO("I2C Interface is initialized!");
  // Loop duration, rate does not work here since it interferes with the ioctl
  double frequency;
  if (!nhp.getParam("system/frequency", frequency)) ROS_ERROR("Could not find system/frequency parameter!");
  ros::Rate loop_rate(frequency);
  // Define publisher
  // ros::Publisher systemControl_pub = nh.advertise<interfaces::SystemControl>("systemControl", 1000);
  // Open the I2C bus
  int file_i2c;
  int length = 2;				// number of bytes to read
  unsigned char buffer[60];
  char *filename = (char*)"/dev/i2c-1";
  if ((file_i2c = open(filename, O_RDWR)) < 0)
  {
	  //ERROR HANDLING
	  ROS_ERROR("Failed to open the i2c bus");
	  exit(1);
  }
  int addr = 0x52;		// the i2c adress of the slave
  if (ioctl(file_i2c, I2C_SLAVE, addr) < 0) {
	  ROS_ERROR("Failed to acquire bus access and/or talk to slave.");
	  exit(1);
  }
  // Wait a few seconds before everything is initialized
  sleep(2);
  // Start the ROS loop
  while (ros::ok())
  {
	  // Read data from i2c bus
	  I2CRead(length, file_i2c, buffer);
	  // Debugging
	  ROS_INFO("%x %x", buffer[0], buffer[1]);
	  // Sleep until rate is reached
	  loop_rate.sleep();
  }
  // Close device
  close(file_i2c);
  return 0;
}
