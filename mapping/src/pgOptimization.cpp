/*
	Node for the loop closure detection, TODO: Change everything to eigen vectors and matrices to make it better readable!
*/

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
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
		void callbackPG(const mapping::PG::ConstPtr& msg_in);									// callback function to receive the pose graph information
		void callbackLC(const mapping::LC::ConstPtr& msg_in);									// callback to receive the loop closure information
		Listener(ros::NodeHandle nh, ros::NodeHandle nhp);										// this is the constructor
	private:
		// Storage capacity for the pose graph information, TODO: Change to dynamic arrays?
		MatrixXd PG;
		MatrixXd PG_opt;
		MatrixXi LC;
		unsigned int idx_PG;
		unsigned int idx_LC;

		// ...
		double dx;

		// Subscriber and Publisher
		ros::Subscriber sub_PG;
		ros::Subscriber sub_LC;
		ros::Publisher pub_Map;

		// functions
		Vector3d getRelMeasurement(Vector3d x_i, Vector3d x_j);
		Matrix3d getJacobianA(Vector3d z, Vector3d p_i, Vector3d p_j);
		Matrix3d getJacobianB(Vector3d z, Vector3d p_i, Vector3d p_j);
		void optimizePoseGraph();
		nav_msgs::OccupancyGrid generateGridMap();
};

// Constructor
Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	idx_PG = 0;
	idx_LC = 0;

	// Initialize variables
	dx = pow(10,-9);

	// Initialize subscribers
	sub_PG = nh.subscribe("poseGraph", 1000, &Listener::callbackPG, this);
	sub_LC = nh.subscribe("loopClosures", 1000, &Listener::callbackLC, this);
	// Initialize Publisher
	pub_Map = nh.advertise<nav_msgs::OccupancyGrid>("occupancyGrid", 10);
}

// Callback functions
void Listener::callbackPG(const mapping::PG::ConstPtr& msg_in)
{
	mapping::PG msg_tmp = *msg_in;
	Vector3d new_PG; new_PG << msg_tmp.x, msg_tmp.y, msg_tmp.phi;

	PG.conservativeResize(3,idx_PG+1);
	PG_opt = MatrixXd::Zero(3,idx_PG+1);

	PG.col(idx_PG) = new_PG;
	PG_opt = PG;

	idx_PG++;

	if (idx_LC > 1) {
		ROS_INFO("pgOptimization: Start Pose Graph Optimization!");
		optimizePoseGraph();
		ROS_INFO("pgOptimization: Pose Graph Optimization finished!");
	}
	nav_msgs::OccupancyGrid map = generateGridMap();
	pub_Map.publish(map);
}
void Listener::callbackLC(const mapping::LC::ConstPtr& msg_in)
{
	mapping::LC msg_tmp = *msg_in;
	Vector2i new_LC; new_LC << msg_tmp.idx1, msg_tmp.idx2;

	LC.conservativeResize(2,idx_LC+1);
	LC.col(idx_LC) = new_LC;
	idx_LC++;
}

// Function for optimizing the pose graph
void Listener::optimizePoseGraph() {
	// Define error matrices, TODO: Change that according to the paper!
	double sigma = 0.1;
	Matrix3d Omega; Omega << sigma, 0, 0, 0, sigma, 0, 0, 0, sigma;

	// Generate relative odometry measurements
	MatrixXd Z; Z = MatrixXd::Zero(3,idx_PG-1);
	for(int i=1; i<idx_PG; i++) {
		Z.block(0,i-1,3,1) = getRelMeasurement(PG.col(i-1), PG.col(i));
	}

	// Iterate until convergence
	double err = 1000000;
	int iter = 0;
	while(err > 0.001 && iter < 100) {
		VectorXd b(3*idx_PG); b = VectorXd::Zero(3*idx_PG);
		MatrixXd H(3*idx_PG,3*idx_PG); H = MatrixXd::Zero(3*idx_PG,3*idx_PG);
		// Add odometry measurements
		for(int i=1; i<idx_PG; i++) {
			Matrix3d A = getJacobianA(Z.col(i-1), PG_opt.col(i-1), PG_opt.col(i));
			Matrix3d B = getJacobianB(Z.col(i-1), PG_opt.col(i-1), PG_opt.col(i));
			H.block(i-1,i-1,3,3) += A.transpose() * Omega * A;
			H.block(i-1,i,3,3) += A.transpose() * Omega * B;
			H.block(i,i-1,3,3) += B.transpose() * Omega * A;
			H.block(i,i,3,3) += B.transpose() * Omega * B;

			Vector3d e = Z.col(i-1) - getRelMeasurement(PG_opt.col(i-1), PG_opt.col(i));
			b.segment(i-1,3) += A.transpose() * Omega * e;
			b.segment(i,3) += B.transpose() * Omega * e;

			// ROS_INFO("PG_opt:\n %f, %f, %f", PG_opt(0,i-1), PG_opt(1,i-1), PG_opt(2,i-1));
			// ROS_INFO("A:\n %f, %f, %f \n %f, %f, %f \n %f, %f, %f", A(0,0), A(0,1), A(0,2), A(1,0), A(1,1), A(1,2), A(2,0), A(2,1), A(2,2));
			// ROS_INFO("B:\n %f, %f, %f \n %f, %f, %f \n %f, %f, %f", B(0,0), B(0,1), B(0,2), B(1,0), B(1,1), B(1,2), B(2,0), B(2,1), B(2,2));
		}
		// Add loop closure measurements
		for(int i=0; i<idx_LC; i++) {
			Vector3d z_LC = Vector3d::Zero();
			int idx_i = LC(0,i)-1;
			int idx_j = LC(1,i)-1;

			Matrix3d A = getJacobianA(z_LC, PG_opt.col(idx_i), PG_opt.col(idx_j));
			Matrix3d B = getJacobianB(z_LC, PG_opt.col(idx_i), PG_opt.col(idx_j));
			H.block(idx_i,idx_i,3,3) += A.transpose() * Omega * A;
			H.block(idx_i,idx_j,3,3) += A.transpose() * Omega * B;
			H.block(idx_j,idx_i,3,3) += B.transpose() * Omega * A;
			H.block(idx_j,idx_j,3,3) += B.transpose() * Omega * B;

			Vector3d e = z_LC - getRelMeasurement(PG_opt.col(idx_i), PG_opt.col(idx_j));
			b.segment(idx_i,3) += A.transpose() * Omega * e;
			b.segment(idx_j,3) += B.transpose() * Omega * e;
		}
		// Keep the first node fixed
		H.block(0,0,3,3) += Matrix3d::Identity();
		// Solve the problem
		VectorXd dx = H.colPivHouseholderQr().solve(b);
		// Determine correction error
		err = dx.norm();
		iter++;
		double rel_err = (H*dx + b).norm() / b.norm();
		ROS_INFO("n_norm: %f", b.norm());
		ROS_INFO("rel_err: %f", rel_err);
		ROS_INFO("err: %f", err);
		// Update the pose
		MatrixXd PG_add; PG_add = MatrixXd::Zero(3,idx_PG);
		for(int i=0; i<idx_PG; i++) {
			PG_add.col(i) = dx.segment(3*i,3);
		}
		PG_opt += PG_add;
	}
}

// Calculate Jacobian
Matrix3d Listener::getJacobianA(Vector3d z, Vector3d p_i, Vector3d p_j) {
	// Define variables
	Matrix3d J = Matrix3d::Zero();

	// Get the current error
	Vector3d e = z - getRelMeasurement(p_i, p_j);
	// ROS_INFO("e: %f, %f, %f", e(0), e(1), e(2));

	// Get the jacobian
	Vector3d p_i_temp;
	for(int i=0; i<3; i++) {
		p_i_temp = p_i;
		p_i_temp(i) = p_i_temp(i) + dx;
		J.block(0,i,3,1) = (z - getRelMeasurement(p_i_temp, p_j) - e) / dx;
	}

	return J;
}

// Calculate Jacobian
Matrix3d Listener::getJacobianB(Vector3d z, Vector3d p_i, Vector3d p_j) {
	// Define variables
	Matrix3d J = Matrix3d::Zero();

	// Get the current error
	Vector3d e = z - getRelMeasurement(p_i, p_j);

	// Get the jacobian
	Vector3d p_j_temp;
	for(int i=0; i<3; i++) {
		p_j_temp = p_j;
		p_j_temp(i) = p_j_temp(i) + dx;
		J.block(0,i,3,1) = (z - getRelMeasurement(p_i, p_j_temp) - e) / dx;
	}

	return J;
}

// Function for calculating relative measurements
Vector3d Listener::getRelMeasurement(Vector3d p_i, Vector3d p_j) {
	Vector2d v;
	v << (p_j(0) - p_i(0)), (p_j(1) - p_i(1));
	Matrix2d R;
	R << cosf(p_i(2)), -sinf(p_i(2)), sinf(p_i(2)), cosf(p_i(2));
	Vector2d v_rel = R.transpose() * v;

 	Vector3d msg_out;
	msg_out(0) = v_rel(0);
	msg_out(1) = v_rel(1);

	double deltaAngle = (p_j(2) - p_i(2));
	if(deltaAngle > PI) {
		deltaAngle = 2*PI - deltaAngle;
	} else if(deltaAngle < -PI) {
		deltaAngle = 2*PI + deltaAngle;
	}
	msg_out(2) = deltaAngle;

	return msg_out;
}

// Generate the OccupancyGrid based on the optimized poses
nav_msgs::OccupancyGrid Listener::generateGridMap() {
	// Define message variable
	nav_msgs::OccupancyGrid map;
	// Get map information
	float boundaryDistance = 1;		// [m]
	float x_min = PG_opt.row(0).minCoeff();
	float x_max = PG_opt.row(0).maxCoeff();
	float y_min = PG_opt.row(1).minCoeff();
	float y_max = PG_opt.row(1).maxCoeff();
	// Define map meta data
	map.info.resolution = 0.1;		// [m/cells]
	map.info.width = (int)((x_max - x_min + 2.0f * boundaryDistance) / map.info.resolution);
	map.info.height = (int)((y_max - y_min + 2.0f * boundaryDistance) / map.info.resolution);
	map.info.origin.position.x = x_min - 1.0f;							//origin : The 2-D pose of the lower-left pixel in the map
	map.info.origin.position.y = y_min - 1.0f;
	map.info.origin.orientation.w = 1.0f;
	// Resize the data structure
	map.data.resize(map.info.width*map.info.height);
	// Go over poses, starting with the pose of the first Loop Closure and ending with the last loop closure
	// int idx_min = LC.minCoeff();
	// int idx_max = LC.maxCoeff();
	// for(int i=idx_min-1; i<idx_max; i++) {
	for(int i=0; i<idx_PG-1; i++) {
		// Define the vector, TODO: Use Bresenham's line algorithm or something similar
		Vector2f v; v << PG_opt(0,i+1) - PG_opt(0,i), PG_opt(1,i+1) - PG_opt(1,i);
		Vector2f x; x << PG_opt(0,i), PG_opt(1,i);
		Vector2f y;
		for(int j=0; j<1001; j++) {
			y = x + v * (((float)j)/1000.0f);
			int idx_x = (int)((y(0) - x_min + boundaryDistance) / map.info.resolution);
			int idx_y = (int)((y(1) - y_min + boundaryDistance) / map.info.resolution);
			int idx = idx_x + idx_y*map.info.width;
			map.data[idx] = (unsigned char)0x64;
		}
	}
	// Return the map
	return map;
}

int main(int argc, char **argv)
{
	// Initialize the ROS System
	ros::init(argc, argv, "pgOptimization");
	// Initialize node handle
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	// Initialize the Listener
	Listener listener(nh, nhp);
	// Start the loop
	ros::spin();
	return 0;
}
