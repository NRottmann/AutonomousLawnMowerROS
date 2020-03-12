/*
	Node for the loop closure detection, TODO: Change everything to eigen vectors and matrices to make it better readable!
*/

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
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
		MatrixXd PG_closed;					// Temporary closed graph array
		MatrixXi LC;
		unsigned int idx_PG;
		unsigned int idx_LC;

		// Differential and learning rate
		double delta;
		double alpha;

		// Subscriber and Publisher
		ros::Subscriber sub_PG;
		ros::Subscriber sub_LC;
		ros::Publisher pub_Map;
		ros::Publisher pub_PGrviz;

		// functions
		Vector3d getRelMeasurement(Vector3d x_i, Vector3d x_j);
		Matrix3d getJacobianA(Vector3d z, Vector3d p_i, Vector3d p_j);
		Matrix3d getJacobianB(Vector3d z, Vector3d p_i, Vector3d p_j);
		void optimizePoseGraph();
		bool inPolygon(int n_vertices, double x, double y);
		bool intersect(Vector2d A, Vector2d B, Vector2d C, Vector2d D);
		Vector2d intersectPoint(Vector2d A, Vector2d B, Vector2d C, Vector2d D);
		double cross2D(Vector2d A, Vector2d B);
		void publishLineSegments();
		nav_msgs::OccupancyGrid generateGridMap();
};

// Constructor
Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	idx_PG = 0;
	idx_LC = 0;

	// Initialize variables
	delta = pow(10,-9);
	alpha = pow(10,-1);

	// Initialize subscribers
	sub_PG = nh.subscribe("poseGraph", 1000, &Listener::callbackPG, this);
	sub_LC = nh.subscribe("loopClosures", 1000, &Listener::callbackLC, this);
	// Initialize Publisher
	pub_Map = nh.advertise<nav_msgs::OccupancyGrid>("occupancyGrid", 1, true);			// Publishers with latch
	pub_PGrviz = nh.advertise<visualization_msgs::Marker>("visualizationPG", 1, true);
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

	if (idx_LC > 0) {
		ROS_INFO("pgOptimization: Start Pose Graph Optimization!");
		optimizePoseGraph();
		ROS_INFO("pgOptimization: Pose Graph Optimization finished!");

		// Generate Grid map and pulish it
		ROS_INFO("pgOptimization: Start Map Generation!");
		nav_msgs::OccupancyGrid map = generateGridMap();
		pub_Map.publish(map);
		ROS_INFO("pgOptimization: Map Generation finished!");
	}

	// Publish the visualization of the pose graph
	publishLineSegments();
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
	double sigma_LC = 0.1;
	Matrix3d Omega; Omega << sigma, 0, 0, 0, sigma, 0, 0, 0, sigma;
	Matrix3d Omega_LC; Omega_LC << sigma_LC, 0, 0, 0, sigma_LC, 0, 0, 0, sigma_LC;

	// Generate relative odometry measurements
	MatrixXd Z; Z = MatrixXd::Zero(3,idx_PG-1);
	for(int i=1; i<idx_PG; i++) {
		Z.block(0,i-1,3,1) = getRelMeasurement(PG.col(i-1), PG.col(i));
	}

	// Iterate until convergence
	double err = 1000000;
	int iter = 0;
	while(err > 0.01 && iter < 100) {
		VectorXd b(3*idx_PG); b = VectorXd::Zero(3*idx_PG);
		MatrixXd H(3*idx_PG,3*idx_PG); H = MatrixXd::Zero(3*idx_PG,3*idx_PG);
		// Add odometry measurements
		for(int i=1; i<idx_PG; i++) {
			Matrix3d A = getJacobianA(Z.col(i-1), PG_opt.col(i-1), PG_opt.col(i));
			Matrix3d B = getJacobianB(Z.col(i-1), PG_opt.col(i-1), PG_opt.col(i));
			H.block((i-1)*3,(i-1)*3,3,3) += (A.transpose() * Omega * A);
			H.block((i-1)*3,i*3,3,3) += (A.transpose() * Omega * B);
			H.block(i*3,(i-1)*3,3,3) += (B.transpose() * Omega * A);
			H.block(i*3,i*3,3,3) += (B.transpose() * Omega * B);

			Vector3d e = Z.col(i-1) - getRelMeasurement(PG_opt.col(i-1), PG_opt.col(i));
			b.segment((i-1)*3,3) += (A.transpose() * Omega * e);
			b.segment(i*3,3) += (B.transpose() * Omega * e);
		}
		// Add loop closure measurements
		for(int i=0; i<idx_LC; i++) {
			Vector3d z_LC = Vector3d::Zero();
			int idx_i = LC(0,i)-1;
			int idx_j = LC(1,i)-1;

			Matrix3d A = getJacobianA(z_LC, PG_opt.col(idx_i), PG_opt.col(idx_j));
			Matrix3d B = getJacobianB(z_LC, PG_opt.col(idx_i), PG_opt.col(idx_j));
			H.block(idx_i*3,idx_i*3,3,3) += (A.transpose() * Omega_LC * A);
			H.block(idx_i*3,idx_j*3,3,3) += (A.transpose() * Omega_LC * B);
			H.block(idx_j*3,idx_i*3,3,3) += (B.transpose() * Omega_LC * A);
			H.block(idx_j*3,idx_j*3,3,3) += (B.transpose() * Omega_LC * B);

			Vector3d e = z_LC - getRelMeasurement(PG_opt.col(idx_i), PG_opt.col(idx_j));
			b.segment(idx_i*3,3) += (A.transpose() * Omega_LC * e);
			b.segment(idx_j*3,3) += (B.transpose() * Omega_LC * e);
		}
		// Keep the first node fixed
		H.block(0,0,3,3) += Matrix3d::Identity();
		// Solve the problem
		VectorXd dx = H.colPivHouseholderQr().solve(-b);
		// Determine correction error
		err = dx.norm();
		iter++;

		ROS_INFO("err: %f", err);

		// Update the pose
		MatrixXd PG_add; PG_add = MatrixXd::Zero(3,idx_PG);
		for(int i=0; i<idx_PG; i++) {
			PG_add.col(i) = dx.segment(3*i,3);
		}
		PG_opt += alpha * PG_add;
	}
}

// Calculate Jacobian
Matrix3d Listener::getJacobianA(Vector3d z, Vector3d p_i, Vector3d p_j) {
	// Define variables
	Matrix3d J = Matrix3d::Zero();

	// Get the current error
	Vector3d e = z - getRelMeasurement(p_i, p_j);

	// Get the jacobian
	Vector3d p_i_temp;
	for(int i=0; i<3; i++) {
		p_i_temp = p_i;
		p_i_temp(i) = p_i_temp(i) + delta;
		J.block(0,i,3,1) = ((z - getRelMeasurement(p_i_temp, p_j)) - e) / delta;
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
		p_j_temp(i) = p_j_temp(i) + delta;
		J.block(0,i,3,1) = ((z - getRelMeasurement(p_i, p_j_temp)) - e) / delta;
	}

	return J;
}

// Function for calculating relative measurements
Vector3d Listener::getRelMeasurement(Vector3d p_i, Vector3d p_j) {
	Vector2d v;
	v << (p_j(0) - p_i(0)), (p_j(1) - p_i(1));
	Matrix2d R;
	R << cos(p_i(2)), -sin(p_i(2)), sin(p_i(2)), cos(p_i(2));
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

	// Close the map by simply adding a line between each LC, TODO: Can we do better?
	MatrixXi occupancyMatrix = MatrixXi::Zero(map.info.width,map.info.height);
	for (int countClosed=0; countClosed < idx_LC; countClosed++) {
		int idx_start = LC(0,countClosed); int idx_end = LC(1,countClosed);
		int n_vertices = idx_end-idx_start+1;
		PG_closed = MatrixXd::Zero(3,n_vertices);
		PG_closed = PG_opt.block(0,idx_start-1,3,n_vertices);
		PG_closed.block(0,n_vertices-1,3,1) = PG_closed.block(0,0,3,1);
		// Check which points are inside the polygon
		double x0 = x_min - boundaryDistance;
		double y0 = y_min - boundaryDistance;
		for(int jx=0; jx<map.info.width; jx++) {
			for(int jy=0; jy<map.info.height; jy++) {
				if(inPolygon(n_vertices, x0+jx*map.info.resolution, y0+jy*map.info.resolution)) {
					occupancyMatrix(jx,jy)++;
				}
			}
		}
	}
	// Put data into occupancyGrid format
	for(int jx=0; jx<map.info.width; jx++) {
		for(int jy=0; jy<map.info.height; jy++) {
			int idx = jx + jy*map.info.width;
			if(occupancyMatrix(jx,jy) >= (int)(idx_LC/2)) {
				map.data[idx] = 0x64;
			} else {
				map.data[idx] = 0x00;
			}
		}
	}

	// Go over poses, starting with the pose of the first Loop Closure and ending with the last loop closure
	/*int idx_min = LC.minCoeff();
	int idx_max = LC.maxCoeff();
	for(int i=idx_min-1; i<idx_max; i++) {
		// Create the vector between the DPs
		Vector2d v; v << PG_opt(0,i+1) - PG_opt(0,i), PG_opt(1,i+1) - PG_opt(1,i);
		Matrix2d Mv = Matrix2d::Zero(); Mv.block(0,0,2,1) = v;
		// Deine minum map values
		double x0 = x_min - boundaryDistance;
		double y0 = y_min - boundaryDistance;
		for(int jx=0; jx<map.info.width; jx++) {
			for(int jy=0; jy<map.info.height; jy++) {
				// Check all positions
				Vector2d z; z << x0+jx*map.info.resolution, y0+jy*map.info.resolution;
				Mv.block(0,1,2,1) = z;
				int idx = jx + jy*map.info.width;
				if (std::signbit(Mv.determinant())) {
					map.data[idx] += 0;
				} else {
					map.data[idx] += 1;
				}
			}
		}
	}
	// Go through all entries of the grid map
	int maxCoeff = 0;
	for(int jx=0; jx<map.info.width; jx++) {
		for(int jy=0; jy<map.info.height; jy++) {
			// Check all positions
			int idx = jx + jy*map.info.width;
			if (maxCoeff < map.data[idx]) {
				maxCoeff = map.data[idx];
			}
			if (map.data[idx] > (idx_max-idx_min+1)/2) {
				map.data[idx] = (unsigned char)0x64;
			} else {
				map.data[idx] = (unsigned char)0x00;
			}
		}
	}

	/* for(int i=0; i<idx_PG-1; i++) {
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
	} */
	// Return the map
	return map;
}

// Check if a point
bool Listener::inPolygon(int n_vertices, double x, double y) {
	int i, j;
	bool c = false;
  for (i = 0, j = n_vertices-1; i < n_vertices; j = i++) {
    if ( ((PG_closed(1,i) > y) != (PG_closed(1,j) > y)) &&
     (x < (PG_closed(0,j) - PG_closed(0,i)) * (y - PG_closed(1,i)) / (PG_closed(1,j) - PG_closed(1,i)) + PG_closed(0,i)) )
       c = !c;
  }
  return c;
}

bool Listener::intersect(Vector2d A, Vector2d B, Vector2d C, Vector2d D) {
	// Check if the line segments AB and CD intersect
	// (1): Start by checking if line segments are co-linear
	if (cross2D(B-A,C-A) == 0 && cross2D(B-A,D-A) == 0) {
		return false;
	} else {
		if (cross2D(B-A,C-A)*cross2D(B-A,D-A) <= 0 && cross2D(D-C,A-C)*cross2D(D-C,B-C) <= 0) {
			return true;
		} else {
			return false;
		}
	}
}

Vector2d Listener::intersectPoint(Vector2d A, Vector2d B, Vector2d C, Vector2d D) {
	// If line segments AB and CD intersect, we can calculate the intersection point
	double t = cross2D(A-C,D-C) / cross2D(D-C,B-A);
	return ((B-A)*t + A);
}

double Listener::cross2D(Vector2d A, Vector2d B) {
	// Calculate 2D cross product
	return (A(0)*B(1) - A(1)*B(0));
}

// Create a visualization message
void Listener::publishLineSegments() {
	visualization_msgs::Marker PG_lines, LC_lines;
	PG_lines.ns = "PG";
	PG_lines.action = visualization_msgs::Marker::ADD;
	PG_lines.pose.orientation.w = 1.0;
	PG_lines.id = 0;
	PG_lines.type = visualization_msgs::Marker::LINE_STRIP;
	PG_lines.scale.x = 0.1;
	PG_lines.color.b = 1.0;				// Lines are red
	PG_lines.color.a = 1.0;
	PG_lines.header.frame_id = "/map";

	LC_lines.ns = "PG";
	LC_lines.action = visualization_msgs::Marker::ADD;
	LC_lines.pose.orientation.w = 1.0;
	LC_lines.id = 1;
	LC_lines.type = visualization_msgs::Marker::LINE_LIST;
	LC_lines.scale.x = 0.1;
	LC_lines.color.r = 1.0;				// Lines are red
	LC_lines.color.a = 1.0;
	LC_lines.header.frame_id = "/map";

	// Fill up the Pose Graph
	for(int i=0; i<idx_PG; i++) {
		geometry_msgs::Point p;
		p.x = PG_opt(0,i);
		p.y = PG_opt(1,i);
		p.z = 0.0;
		PG_lines.points.push_back(p);
	}

	// Fill up the Loop Closures
	for(int i=0; i<idx_LC; i++) {
		geometry_msgs::Point p1, p2;
		p1.x = PG_opt(0,LC(0,i)-1);
		p1.y = PG_opt(1,LC(0,i)-1);
		p1.z = 0.0;
		p2.x = PG_opt(0,LC(1,i)-1);
		p2.y = PG_opt(1,LC(1,i)-1);
		p2.z = 0.0;
		LC_lines.points.push_back(p1);
		LC_lines.points.push_back(p2);
	}

	// Publish the messages
	pub_PGrviz.publish(PG_lines);
	pub_PGrviz.publish(LC_lines);
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
