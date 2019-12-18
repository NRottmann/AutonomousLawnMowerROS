/*
	Node for the loop closure detection, TODO: Change everything to eigen vectors and matrices to make it better readable!
*/

#include <ros/ros.h>
#include <mapping/DP.h>
#include <mapping/LC.h>
#include <mapping/PG.h>
#include <Eigen/Eigen>

#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

using namespace Eigen;

// Define callback class to listen to the Odometry
class Listener
{
	public:
		void callbackDP(const mapping::DP::ConstPtr& msg_in);									// callback function to receive the dominant points
		Listener(ros::NodeHandle nh, ros::NodeHandle nhp);										// this is the constructor
	private:
		// Storage capacity for the dominant points
		MatrixXf DP;							// Dynamic storage array for the DPs
		unsigned int idx;					// Index of the current DP

		// Storage capacity for lengths and angles
		VectorXf length;
		VectorXf angle;
		VectorXf varPhi;

		// Storage capacity for the angle vectors for comparison
	 	MatrixXf theta;
		VectorXf x;

		// Correlation Matrix
		MatrixXf C;

		// Parameter
		float L_NH;																														// Neighborhood Length
		float c_min;																													// Minimum correlation
		unsigned int m;																												// Number of evaluation points

		// ...
		bool foundCorrelationStart;
		unsigned int correlationIdx;
		unsigned int correlationIdxStart;

		// functions
		float getAngle(float x);

		// Subscriber and Publisher
		ros::Subscriber sub_DP;
		ros::Publisher pub_LC;
		ros::Publisher pub_PG;
};
// Constructor
Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Get Parameters
	if (!nhp.getParam("mapping/L_NH", L_NH)) ROS_ERROR("lcDetection: Could not find mapping/L_NH parameter!");
	ROS_INFO("lcDetection: Loaded Parameter\n L_NH: %f", L_NH);
	if (!nhp.getParam("mapping/c_min", c_min)) ROS_ERROR("lcDetection: Could not find mapping/c_min parameter!");
	ROS_INFO("lcDetection: Loaded Parameter\n c_min: %f", c_min);
	/* if (!nhp.getParam("mapping/m", m)) ROS_ERROR("lcDetection: Could not find mapping/m parameter!");
	ROS_INFO("lcDetection: Loaded Parameter\n m: %f", m); */
	m = 30;

	// The evaluation positions
	x = VectorXf::Zero(m);
	for(int i=0; i<m; i++) {
		x(i) = -L_NH + ((float)i/(float)(m-1)) * 2.0f * L_NH;
	}

	// Initialize vector array
	theta = MatrixXf::Zero(m,1);

	// Initialize Correlation Matrix
	C = MatrixXf::Zero(1,1); C(0,0) = 1000;

	// Initialize variables
	idx = 1;
	DP = MatrixXf::Zero(2,1);
	length = VectorXf::Zero(1);
	varPhi = VectorXf::Zero(1);

	// Indices to find the correlation starting point
	foundCorrelationStart = false;
	correlationIdx = 0;
	correlationIdxStart = 0;

	// Initialize subscribers
	sub_DP = nh.subscribe("dominantPoints", 1000, &Listener::callbackDP, this);
	// Initialize publisher
	pub_LC = nh.advertise<mapping::LC>("loopClosures", 1000);
	pub_PG = nh.advertise<mapping::PG>("poseGraph", 1000);
}

// Callback functions
void Listener::callbackDP(const mapping::DP::ConstPtr& msg_in)
{
	// Allocate DP message
	mapping::DP msg_tmp = *msg_in;
  DP.conservativeResize(2,idx+1);
	DP(0,idx) = msg_tmp.x; DP(1,idx) = msg_tmp.y;

	// Resize Correlation Matrix
	C.conservativeResize(idx+1,idx+1);
	for(int i=0; i<idx+1; i++) {
		C(i,idx) = 1000; C(idx,i) = 1000;
	}

	// Resize theta
	theta.conservativeResize(m,idx+1);
	for(int i=0; i<m; i++) {
		theta(i,idx) = 0;
	}

	// Generate vector between DPs
	Vector2f v = DP.col(idx) - DP.col(idx-1);

	// Calculate length and angle difference between DPs
	length.conservativeResize(idx+1);
	varPhi.conservativeResize(idx+1);
	length(idx) = length(idx-1) + v.norm();
	varPhi(idx) = atan2f(v(1),v(0));

	// Check if the difference completed a full circle
	float deltaAngle = varPhi(idx) - varPhi(idx-1);
	if(deltaAngle > PI) {
		deltaAngle = 2*PI - deltaAngle;
	} else if(deltaAngle < -PI) {
		deltaAngle = 2*PI + deltaAngle;
	}

	// Add up the angles
	angle.conservativeResize(idx);
	if(idx == 1) {
		angle(idx-1) = deltaAngle;
	} else {
		angle(idx-1) = angle(idx-2) + deltaAngle;
	}

	// Publish the poses
	mapping::PG msg_out_PG;
	msg_out_PG.x = DP(0,idx-1);
	msg_out_PG.y = DP(1,idx-1);
	msg_out_PG.phi = angle(idx-1);
	pub_PG.publish(msg_out_PG);

	// Generate angular vectors
	// Start by checking whether we already collected enough data or not
	float deltaL = 0;
	if(!foundCorrelationStart) {
		deltaL = fabs(length(idx-1) - length(0));
		if(deltaL > L_NH) {
			correlationIdx = idx-1;
			correlationIdxStart = correlationIdx;
			foundCorrelationStart = true;
		}
	} else {
		bool foundSomething = true;
		while(foundSomething) {
			deltaL = fabs(length(idx-1) - length(correlationIdx));
			if(deltaL > L_NH) {
				// Generate the angular vector
				for(int i=0; i<m; i++) {
					float l_eva = length(correlationIdx) + x(i);
					theta(i,correlationIdx) = getAngle(l_eva) - angle(correlationIdx);
				}
				// Compare angular vector with the already existing ones and check for loop closures
				for(int i=correlationIdxStart; i<correlationIdx; i++) {
					// Compare angular vectors
					VectorXf v_tmp = theta.col(i) - theta.col(correlationIdx);
					VectorXf v_plot1 = theta.col(i);
					VectorXf v_plot2 = theta.col(correlationIdx);
					C(i,correlationIdx) = (1.0f/(float)m) * v_tmp.squaredNorm();
					// Check for loop closures
					if(C(i, correlationIdx-1) < c_min) {
						// Check if it is also a minimum
						if(C(i, correlationIdx-1) < C(i, correlationIdx-2) && C(i, correlationIdx-1) < C(i, correlationIdx)) {
							// Check if they are far away
							float dist = fabs(length(correlationIdx-1) - length(i));
							if(dist > 2*L_NH) {
								// Publish loop closure
								mapping::LC msg_out;
								msg_out.idx1 = i;
								msg_out.idx2 = correlationIdx-1;
								pub_LC.publish(msg_out);
							}
						}
					}
				}
				// Set variables
				foundSomething = true;
				correlationIdx++;
			} else {
				foundSomething = false;
			}
		}
	}

	// Count up the index
	idx++;
}

// Function for receiving angle information
float Listener::getAngle(float x) {
	for (int i=1; i<idx; i++) {
		if(length(i) > x) {
			return angle(i-1);
		}
	}
}

int main(int argc, char **argv)
{
	// Initialize the ROS System
	ros::init(argc, argv, "lcDetection");
	// Initialize node handle
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	// Initialize the Listener
	Listener listener(nh, nhp);
	// Start the loop
	ros::spin();
	return 0;
}
