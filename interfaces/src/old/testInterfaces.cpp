/*
* This node tests the interfaces for the lawn mower
*/

#include <ros/ros.h>
#include <interfaces/Sensor.h>
#include <interfaces/IMU.h>
#include <interfaces/Odometry.h>

// Callback function for the sensorData
void sensorCallback(const interfaces::Sensor::ConstPtr& msg)
{
	ROS_INFO("Sensor Data: [%i][%i][%i][%i]", msg->l1, msg->l2, msg->r1, msg->r2);
}
// Callback function for the imuData
void imuCallback(const interfaces::IMU::ConstPtr& msg)
{
	ROS_INFO("IMU Data: [%f][%f][%f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
	ROS_INFO("Calibration Status: [%i][%i][%i][%i]", msg->sys, msg->gyro, msg->accel, msg->mag);
}
// Callback function for the right odometry data
void odoCallback(const interfaces::Odometry::ConstPtr& msg)
{
	ROS_INFO("Odo Right Data: [%3.3f][%i][%i]", msg->l_R, msg->i_R, msg->v_R);
	ROS_INFO("Odo Left Data: [%3.3f][%i][%i]", msg->l_L, msg->i_L, msg->v_L);
}

int main(int argc, char** argv) {
	// initialize the node
	ros::init(argc, argv, "testInterfaces");
	ros::NodeHandle nh;
	ROS_INFO("Test for interfaces is initialized!");
	// Define Subscriber
	ros::Subscriber sub_sensor = nh.subscribe("sensorData", 1000, sensorCallback);
	//ros::Subscriber sub_imu = nh.subscribe("imuData", 1000, imuCallback);
	//ros::Subscriber sub_odo = nh.subscribe("odometryData", 1000, odoCallback);
	// Start receiving the messages and publish them
	ros::Rate loop_rate(1); // 1 Hz
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

