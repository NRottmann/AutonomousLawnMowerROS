/*
	Node for the loop closure detection, TODO: Change everything to eigen vectors and matrices to make it better readable!
*/

#include <ros/ros.h>
#include <mapping/DP.h>
#include <mapping/LC.h>
#include <mapping/RM.h>
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
		// Storage capacity for the dominant points, TODO: Change to dynamic arrays?
		mapping::DP DP[1000];
		unsigned int idx;					// Index of the current DP

		// Storage capacity for lengths and angles, TODO: Change to dynamic arrays?
		float length[1000];
		float angle[1000];
		float varPhi[1000];

		// Storage capacity for the angle vectors for comparison, TODO: Change the 30 to m!
	 	MatrixXf theta;
		float x[30];

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
		ros::Publisher pub_RM;
};
// Constructor
Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Get Parameters
	if (!nhp.getParam("mapping/L_NH", L_NH)) ROS_ERROR("lcDetection: Could not find mapping/L_NH parameter!");
	ROS_INFO("lcDetection: Loaded Parameter\n L_NH: %f", L_NH);
	if (!nhp.getParam("mapping/c_min", c_min)) ROS_ERROR("lcDetection: Could not find mapping/c_min parameter!");
	ROS_INFO("lcDetection: Loaded Parameter\n c_min: %f", c_min);
	//if (!nhp.getParam("mapping/m", m)) ROS_ERROR("lcDetection: Could not find mapping/m parameter!");
	//ROS_INFO("lcDetection: Loaded Parameter\n m: %f", m);
	m = 30;

	// The evaluation positions
	for(int i=0; i<30; i++) {
		x[i] = -L_NH + ((float)i/(float)(30-1)) * 2.0f * L_NH;
	}

	// Initialize vector array
	theta = MatrixXf::Zero(30,1000);

	// Initialize variables
	idx = 1;
	DP[0].x = 0.0f;	DP[0].y = 0.0f; DP[0].idx = 0;
	length[0] = 0.0f; varPhi[0] = 0.0f;

	// Initialize correlation array
	C = MatrixXf::Zero(1000,1000);
	for (int i = 0; i < 1000; i++) {
		for (int j = 0; j < 1000; j++) {
		  C(i,j) = 10000;
		}
	}

	foundCorrelationStart = false;
	correlationIdx = 0;
	correlationIdxStart = 0;

	// Initialize subscribers
	sub_DP = nh.subscribe("dominantPoints", 1000, &Listener::callbackDP, this);
	// Initialize publisher
	pub_LC = nh.advertise<mapping::LC>("loopClosures", 1000);
	pub_RM = nh.advertise<mapping::RM>("relativeMeasurements", 1000);
}
// Callback functions
void Listener::callbackDP(const mapping::DP::ConstPtr& msg_in)
{
	// In this function, we generate the length and angles for loop closure detection
	if(idx > 999) {
		ROS_ERROR("lcDetection: Array Size of DP too small!");
	}
  DP[idx] = *msg_in;

	// Generate vector between DPs
	Vector2f v;
	v << DP[idx].x - DP[idx-1].x, DP[idx].y - DP[idx-1].y;

	// Calculate length and angle difference between DPs
	length[idx] = length[idx-1] + v.norm();
	varPhi[idx] = atan2f(v(1),v(0));

	// Check if the difference completed a full circle
	float deltaAngle = varPhi[idx] - varPhi[idx-1];
	if(deltaAngle > PI) {
		deltaAngle = 2*PI - deltaAngle;
	} else if(deltaAngle < -PI) {
		deltaAngle = 2*PI + deltaAngle;
	}

	// Publish the relative measurements
	Matrix2f R;
	R << cosf(varPhi[idx-1]), -sinf(varPhi[idx-1]), sinf(varPhi[idx-1]), cosf(varPhi[idx-1]);
	Vector2f v_rel = R.transpose() * v;
	mapping::RM msg_out_RM;
	msg_out_RM.x = v_rel(0);
	msg_out_RM.y = v_rel(1);
	msg_out_RM.phi = deltaAngle;
	pub_RM.publish(msg_out_RM);

	// Add up the angles
	if(idx == 1) {
		angle[idx-1] = deltaAngle;
	} else {
		angle[idx-1] = angle[idx-2] + deltaAngle;
	}

	// Generate angular vectors
	// Start by checking whether we already collected enough data or not
	float deltaL = 0;
	if(!foundCorrelationStart) {
		deltaL = fabs(length[idx-1] - length[0]);
		if(deltaL > L_NH) {
			correlationIdx = idx-1;
			correlationIdxStart = correlationIdx;
			foundCorrelationStart = true;
		}
	} else {
		bool foundSomething = true;
		while(foundSomething) {
			deltaL = fabs(length[idx-1] - length[correlationIdx]);
			if(deltaL > L_NH) {
				// Generate the angular vector
				for(int i=0; i<m; i++) {
					float l_eva = length[correlationIdx] + x[i];
					theta(i,correlationIdx) = getAngle(l_eva) - angle[correlationIdx];
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
							float dist = fabs(length[correlationIdx-1] - length[i]);
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
		if(length[i] > x) {
			return angle[i-1];
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
