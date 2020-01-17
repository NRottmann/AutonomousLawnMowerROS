#include "ros/ros.h"
#include "std_msgs/String.h"
#include <interfaces/Sensor.h>
#include <sensor_msgs/Image.h>

// Define a callback class
class SensorListener
{
	public:
		void callbackMid(const sensor_msgs::Image::ConstPtr& msg);
		void callbackRight(const sensor_msgs::Image::ConstPtr& msg);
		void callbackLeft(const sensor_msgs::Image::ConstPtr& msg);
		void publishData();
		SensorListener(ros::NodeHandle nh, ros::NodeHandle nhp);
	private:
		// Sensor data
		uint8_t mid, right, left;
		// Subscriber and Publisher
		ros::Subscriber sub_image_mid;
		ros::Subscriber sub_image_right;
		ros::Subscriber sub_image_left;
		ros::Publisher pub_sensor;
};
// Constructor
SensorListener::SensorListener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Initialize sensor data
	mid = 0; right = 0; left = 0;
	// Initialize subscribers
	sub_image_mid = nh.subscribe("camera_mid/image_raw", 10, &SensorListener::callbackMid, this);
	sub_image_right = nh.subscribe("camera_right/image_raw", 10, &SensorListener::callbackRight, this);
	sub_image_left = nh.subscribe("camera_left/image_raw", 10, &SensorListener::callbackLeft, this);
	// Initialize publisher
	pub_sensor = nh.advertise<interfaces::Sensor>("sensorData", 10);
}


// Callback functions to get the data from the topics
void SensorListener::callbackMid(const sensor_msgs::Image::ConstPtr& msg_in)
{
	uint8_t R = msg_in->data[0];
	uint8_t G = msg_in->data[1];
	uint8_t B = msg_in->data[2];
	mid = 0;
	if ((G > R) && (G > B)) {
		mid = 255;
	}
}
void SensorListener::callbackRight(const sensor_msgs::Image::ConstPtr& msg_in)
{
	uint8_t R = msg_in->data[0];
	uint8_t G = msg_in->data[1];
	uint8_t B = msg_in->data[2];
	right = 0;
	if ((G > R) && (G > B)) {
		right = 255;
	}
}
void SensorListener::callbackLeft(const sensor_msgs::Image::ConstPtr& msg_in)
{
	uint8_t R = msg_in->data[0];
	uint8_t G = msg_in->data[1];
	uint8_t B = msg_in->data[2];
	left = 0;
	if ((G > R) && (G > B)) {
		left = 255;
	}
}

// Function for getting the data
void SensorListener::publishData() {
	// Publish
	interfaces::Sensor msg_out;
	msg_out.mid = mid;
	msg_out.right = right;
	msg_out.left = left;
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
	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		// Subscribe using the callback functions
		ros::spinOnce();
		// Publish
		listener.publishData();
		// Sleep until rate is reached
		loop_rate.sleep();
	}
	return 0;
}
