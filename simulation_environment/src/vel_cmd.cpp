#include "ros/ros.h"
#include <interfaces/Control.h>

int main(int argc, char **argv)
{
  // Initialize the ROS System
  ros::init(argc, argv, "controlDataPublisher");
  // Initialize node handle
  ros::NodeHandle nh;
  // Initialize the publisher for the velocity control commands
  ros::Publisher vel_cmd = nh.advertise<interfaces::Control>("controlData", 1000);
  // Loop rate
  ros::Rate loop_rate(20);
  // Start the loop
  while (ros::ok())
  {
	// Initialize increment message
	interfaces::Control msg;
	// Just drive the vehicle in a circle
	msg.v = 0.5;
	msg.w = 0.5;
    // publish message
	vel_cmd.publish(msg);
	// Sleep until rate is reached
    loop_rate.sleep();
  }
  return 0;
}
