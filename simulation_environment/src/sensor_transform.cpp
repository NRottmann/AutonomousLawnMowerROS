#include "ros/ros.h"
#include "std_msgs/String.h"
#include <interfaces/Sensor.h>
#include <sensor_msgs/Image.h>

// Define a callback class
class SensorListener
{
	public:
		void callback(const sensor_msgs::Image::ConstPtr& msg);
		SensorListener(ros::NodeHandle nh, ros::NodeHandle nhp);		
	private:
		// Subscriber and Publisher
		ros::Subscriber sub_image;
		ros::Publisher pub_sensor;
};
// Constructor
SensorListener::SensorListener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Initialize subscribers
	sub_image = nh.subscribe("camera/image_raw", 10, &SensorListener::callback, this);
	// Initialize publisher
	pub_sensor = nh.advertise<interfaces::Sensor>("sensorData", 10);
}


// Callback function to get the data from the topics
void SensorListener::callback(const sensor_msgs::Image::ConstPtr& msg_in)
{
	interfaces::Sensor msg_out;
	uint8_t R = msg_in->data[0];
	uint8_t G = msg_in->data[1];
	uint8_t B = msg_in->data[2];
	msg_out.r1 = 0;
	if ((G > R) && (G > B)) {
		msg_out.r1 = 255;
	}
	pub_sensor.publish(msg_out);
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
