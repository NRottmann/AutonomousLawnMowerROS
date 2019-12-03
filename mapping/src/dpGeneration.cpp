/*
	Node for the dp generation by driving along the boundary line
*/

#include <ros/ros.h>
#include <interfaces/Odometry.h>
#include <mapping/DP.h>
#include <Eigen/Eigen>

#include <stdio.h>
#include <math.h>

using namespace Eigen;

// Define callback class to listen to the Odometry
class Listener
{
	public:
		void callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in);	// callback function to receive odometry data from the motorInterface
		Listener(ros::NodeHandle nh, ros::NodeHandle nhp);										// this is the constructor
	private:
		interfaces::Odometry msg_odometry;
		// Define vectors and matrices using the Eigen library
		Vector3f pose;																												// Current pose
		Vector2f lastDP;																											// Last DP
		
		float positions[2][1000];																// Storage capacity for positions for the DP generation, TODO: Maybe dynamic allocation?
		unsigned int numPositions;
		
		unsigned int idx;

		// Parameter
		float L;
		float L_min;																													// Minimum length between two DPs
		float e_max;																													// Error bound for line fit

		// Functions
		bool generationDP(Vector2f newPosition);															// Function for the DP generation

		// Subscriber and Publisher
		ros::Subscriber sub_odometry;
		ros::Publisher pub_DP;
};
// Constructor
Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Get Parameters
	if (!nhp.getParam("robot/L", L)) ROS_ERROR("dpGeneration: Could not find robot/L parameter!");
	ROS_INFO("dpGeneration: Loaded Parameter\n L: %f", L);
	if (!nhp.getParam("mapping/L_min", L_min)) ROS_ERROR("dpGeneration: Could not find mapping/L_min parameter!");
	ROS_INFO("dpGeneration: Loaded Parameter\n L_min: %f", L_min);
	if (!nhp.getParam("mapping/e_max", e_max)) ROS_ERROR("dpGeneration: Could not find mapping/e_max parameter!");
	ROS_INFO("dpGeneration: Loaded Parameter\n e_max: %f", e_max);

	// Initialize vectors and matrices with zeros
	pose << 0, 0, 0;
	lastDP << 0, 0;
	positions[0][0] = 0; positions[1][0] = 0;
	
	numPositions = 0;
	idx = 0;

	// Initialize subscribers
	sub_odometry = nh.subscribe("odometryData", 1000, &Listener::callbackOdometry, this);
	// Initialize publisher
	pub_DP = nh.advertise<mapping::DP>("dominantPoints", 1000);
}
// Callback functions
void Listener::callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in)
{
	// In this function the Kalman Filter is implemented
	interfaces::Odometry msg_new = *msg_in;

	// Update the pose
	// TODO: Change this dirty fix
	if (fabs(msg_new.l_L) > 0.2)
		msg_new.l_L = 0.0f;
	if (fabs(msg_new.l_R) > 0.2)
		msg_new.l_R = 0.0f;
	Vector2f deltaOdo;															// holds ds and dphi for the odometry
	deltaOdo << ((msg_new.l_R + msg_new.l_L) / 2),
				((msg_new.l_R - msg_new.l_L) / (2 * L));
	MatrixXf T(3, 2);
	T << cosf(pose(2)), 0,
		   sinf(pose(2)), 0,
		   0, 1;
	pose = pose + T * deltaOdo;

	// Check for new DPs
	Vector2f newPosition;
	newPosition << pose(0), pose(1);
	bool newDP = generationDP(newPosition);
}

// Functions
bool Listener::generationDP(Vector2f newPosition) {
	Vector2f diff = lastDP - newPosition;
	float d = diff.norm();
	// Add new position to the set S (positions)
	numPositions++;
	positions[0][numPositions] = newPosition(0); positions[1][numPositions] = newPosition(1);
	if(d > L_min && numPositions > 1) {
		// Calculate line fit error
		float e = 0;
		float phi = atan2f(diff(1),diff(0));
		for(int i=1; i<(numPositions); i++) {
			Vector2f test;
			test << positions[0][i], positions[1][i];
			test = test - lastDP;
			float testPhi = atan2f(test(1),test(0));
			testPhi = testPhi - phi;
			e += powf(sinf(testPhi) * test.norm(), 2);
		}
		e = e / (numPositions-1);
		// Check if error is over the limit
		if(e > e_max) {
			positions[0][0] = positions[0][numPositions-1]; positions[1][0] = positions[1][numPositions-1];
			positions[0][1] = newPosition(0); positions[1][1] = newPosition(1);
			numPositions = 1;
			idx++;
			
			mapping::DP msg_out;
			msg_out.x = positions[0][0];
			msg_out.y = positions[1][0];
			msg_out.idx = idx;
			pub_DP.publish(msg_out);
			return true;
		}
	}
	return false;
}

int main(int argc, char **argv)
{
	// Initialize the ROS System
	ros::init(argc, argv, "dpGeneration");
	// Initialize node handle
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	// Initialize the Listener
	Listener listener(nh, nhp);
	// Start the loop
	ros::spin();
	return 0;
}
