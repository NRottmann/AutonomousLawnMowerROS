#include "ros/ros.h"
#include "std_msgs/String.h"
#include <interfaces/Chlorophyll.h>
#include <sensor_msgs/Image.h>

// Define a callback class
class SensorListener
{
	public:
		void callback1(const sensor_msgs::Image::ConstPtr& msg);
		SensorListener(ros::NodeHandle nh, ros::NodeHandle nhp);		
	private:
		// Subscriber and Publisher
		ros::Subscriber sub_image1;
		ros::Publisher pub_sensor1;
};
// Constructor
SensorListener::SensorListener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Initialize subscribers
	sub_image1 = nh.subscribe("camera_1/image_raw", 10, &SensorListener::callback1, this);
	// Initialize publisher
	pub_sensor1 = nh.advertise<interfaces::Chlorophyll>("sensor1", 10);
}


// Callback function to get the data from the topics
void SensorListener::callback1(const sensor_msgs::Image::ConstPtr& msg_in)
{
	interfaces::Chlorophyll msg_out;
	msg_out.data = msg_in->data[1];
	pub_sensor1.publish(msg_out);
}

int main(int argc, char **argv)
{
	// Initialize the ROS System
	ros::init(argc, argv, "sensorTransfrom");
	// Initialize node handle
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	// Initialize the publisher for the control commands, tell them which message should be published
	// Initialize the subscriber to get the teleop data
	SensorListener listener(nh, nhp);
	// Loop rate
	ros::Rate loop_rate(30);
	while (ros::ok())
	{
		// Subscribe using the callback functions
		ros::spinOnce();
		// Sleep until rate is reached
		loop_rate.sleep();
	}
	return 0;
}
